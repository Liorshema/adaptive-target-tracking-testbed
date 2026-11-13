"""Compose explicit acceleration bindings into D3; no runtime evaluator or solver."""
import ast
from copy import deepcopy

from robot_configuration.resources import package_config_path
from robot_configuration.yaml_loader import load_yaml
from .equation_expression import parse_expression
from .equation_units import parse_unit


def robot_response_profile():
    """Load owner defaults only when authoring a new experiment snapshot."""
    data=load_yaml(package_config_path(__file__,'robot_simulation','derivative_influences.yaml'))
    from .equation_model import _mapping
    _mapping(data,{'schema_version','profiles'},{'schema_version','profiles'},'influences')
    if type(data['schema_version']) is not int or data['schema_version']!=1:
        raise ValueError('influences: expected schema_version 1')
    return deepcopy(data['profiles']['robot_response'])


def _binding(model, profile):
    """Only direct position/velocity pairs can receive world acceleration unchanged."""
    from .equation_model import _mapping
    if model['frame']!='world' or profile['frame']!='world':
        raise ValueError('Derivative influences require explicit world frame')
    output=model.get('outputs',{})
    if output.get('frame')!='world' or 'position' not in output:
        raise ValueError('Influence requires explicit world physical position output')
    declarations={s['name']:s for s in model['state']}
    tree=parse_expression(output['position'])
    expected={}
    if isinstance(tree,ast.Name):
        p=declarations.get(tree.id,{})
        rate=parse_expression(model['equations'].get(tree.id,'0'))
        if p.get('shape')!=[3] or not isinstance(rate,ast.Name):
            raise ValueError('Influence requires direct position/velocity state bindings')
        expected[rate.id]=[0,1,2]
    elif isinstance(tree,ast.List) and len(tree.elts)==3:
        for index,position in enumerate(tree.elts):
            if not isinstance(position,ast.Name) or declarations.get(position.id,{}).get('shape')!=[]:
                raise ValueError('Influence requires three direct scalar position states')
            rate=parse_expression(model['equations'].get(position.id,'0'))
            if not isinstance(rate,ast.Name) or rate.id in expected:
                raise ValueError('Influence requires distinct direct velocity state derivatives')
            expected[rate.id]=index
    else:
        raise ValueError('Unsupported influence projection: position must be direct world states')
    bindings=profile['bindings']
    if not isinstance(bindings,dict) or bindings!=expected:
        raise ValueError('Acceleration bindings must match the physical position velocity states/components')
    for name,projection in bindings.items():
        spec=declarations.get(name,{})
        if (parse_unit(spec.get('unit','1'))!=parse_unit('m/s') or
            spec.get('shape')!=([3] if isinstance(projection,list) else [])):
            raise ValueError('Acceleration injection requires velocity states with unit m/s')
        if isinstance(projection,list):
            if any(type(i) is not int for i in projection):raise ValueError('Invalid component indices')
        elif type(projection) is not int:raise ValueError('Invalid component index')
    return output['position']


def compose_influences(model, influences):
    """Validate frozen profiles and return ordinary effective D3 equations."""
    from .equation_model import EquationDynamics,_mapping,_name
    from .motion_composition import definition_data
    if not isinstance(influences,list) or len(influences)>4:
        raise ValueError('influences must be a list of at most four frozen profiles')
    base=EquationDynamics(model=model).definition
    result=deepcopy(base);identifiers=set()
    for index,raw in enumerate(influences):
        p=definition_data(raw)
        fields={'schema_version','id','enabled','frame','output','inputs','context',
                'parameters','intermediates','expression','bindings'}
        _mapping(p,fields,fields,'derivative influence')
        if type(p['schema_version']) is not int or p['schema_version']!=1 or type(p['enabled']) is not bool:
            raise ValueError('Influence requires schema_version 1 and boolean enabled')
        identifier=_name(p['id'])
        if identifier in identifiers:raise ValueError('Duplicate influence identity')
        identifiers.add(identifier)
        if p['output']!={'shape':[3],'unit':'m/s^2'}:
            raise ValueError('Influence output must be world acceleration [3] in m/s^2')
        if p['frame'] != 'world':
            raise ValueError('Derivative influences require explicit world frame')
        if not isinstance(p['inputs'],dict) or len(p['inputs'])!=1:
            raise ValueError('Influence requires one explicit physical position input')
        input_name,input_spec=next(iter(p['inputs'].items()));_name(input_name)
        if input_spec!={'source':'outputs.position','shape':[3],'unit':'m','frame':'world'}:
            raise ValueError('Influence position input must bind outputs.position in world metres')
        if p['context']!={'tracking_position_world':{'shape':[3],'unit':'m','frame':'world'}}:
            raise ValueError('Influence requires explicit tracking_position_world context')
        if not isinstance(p['parameters'],dict) or len(p['parameters'])>32 or not isinstance(p['intermediates'],dict) or len(p['intermediates'])>32:
            raise ValueError('Influence parameters/intermediates exceed mapping bounds')
        names=[*p['parameters'],*p['intermediates'],*p['context'],input_name]
        if len(names)!=len(set(names)):raise ValueError('Duplicate influence symbols')
        for name in names:_name(name)
        if not p['enabled']:
            # Validate the frozen profile's own contract without binding it to a
            # target it cannot affect. D3 checks its AST, bounds, units and shape.
            bindings = p['bindings']
            if not isinstance(bindings, dict) or not bindings:
                raise ValueError('Influence bindings must be a nonempty mapping')
            for name in bindings: _name(name)
            projections = list(bindings.values())
            vector = (len(projections) == 1 and projections[0] == [0, 1, 2]
                      and all(type(i) is int for i in projections[0]))
            scalars = (len(projections) == 3 and all(type(i) is int for i in projections)
                       and set(projections) == {0, 1, 2})
            if not (vector or scalars):
                raise ValueError('Influence bindings must project three acceleration components')
            referenced = {node.id for text in [p['expression'], *p['intermediates'].values()]
                          for node in ast.walk(parse_expression(text))
                          if isinstance(node, ast.Name)}
            velocity = 'influenceValidationVelocity'
            while velocity in names or velocity in referenced: velocity += 'x'
            EquationDynamics(model={
                'id': identifier, 'schema_version': 1, 'type': 'equation_state_space',
                'frame': p['frame'],
                'state': [{'name': velocity, 'shape': [3], 'unit': 'm/s',
                           'initial': [0., 0., 0.]}],
                'parameters': p['parameters'], 'intermediates': p['intermediates'],
                'context': {**p['context'], input_name: {
                    key: value for key, value in input_spec.items() if key != 'source'}},
                'equations': {velocity: p['expression']},
            })
            continue
        position=_binding(base,p)
        used=set(result.get('parameters',{}))|set(result.get('intermediates',{}))|set(result.get('context',{}))|{s['name'] for s in result['state']}|{'t','pi'}
        def allocate(stem):
            while stem in used:stem+='x'
            used.add(stem);return stem
        rename={name:allocate(f'influence{index}Symbol{i}') for i,name in enumerate([*p['parameters'],*p['intermediates']])}
        allowed=set(names)|{'t','pi'}
        class Bind(ast.NodeTransformer):
            def visit_Name(self,node):
                if node.id==input_name:return deepcopy(parse_expression(position))
                if node.id not in allowed:
                    raise ValueError(f'Undeclared influence symbol: {node.id}')
                return ast.copy_location(ast.Name(id=rename.get(node.id,node.id),ctx=node.ctx),node)
            def visit_Call(self,node):
                # D3 validates the function allowlist; bind only the arguments.
                return ast.Call(func=deepcopy(node.func),args=[self.visit(a) for a in node.args],keywords=deepcopy(node.keywords))
        def expression(text):return ast.unparse(ast.fix_missing_locations(Bind().visit(parse_expression(text))))
        candidate=deepcopy(result)
        for name,spec in p['parameters'].items():candidate.setdefault('parameters',{})[rename[name]]=spec
        for name,spec in p['context'].items():
            previous=candidate.setdefault('context',{}).get(name)
            if previous is not None and previous!=spec:raise ValueError('Incompatible shared influence context')
            candidate['context'][name]=spec
        for name,text in p['intermediates'].items():candidate.setdefault('intermediates',{})[rename[name]]=expression(text)
        output=allocate(f'influence{index}Acceleration')
        candidate.setdefault('intermediates',{})[output]=expression(p['expression'])
        for name,component in p['bindings'].items():
            term=output if isinstance(component,list) else f'{output}[{component}]'
            candidate['equations'][name]=f"({candidate['equations'][name]}) + {term}"
        # Normal D3 checks parameters, dimensions, shapes, AST costs and domains.
        EquationDynamics(model=candidate)
        if p['enabled']:result=candidate
    return result


def prepare_robot_response(model, influences=None, *, enabled=None):
    """Prepare the single operational UI influence; preserve existing frozen data."""
    from .equation_model import EquationDynamics
    if enabled is not None and type(enabled) is not bool:
        raise ValueError('Robot Response requires boolean opt-in')
    if influences:
        if not isinstance(influences,list) or len(influences)!=1:
            raise ValueError('Robot Response UI supports one explicit influence profile')
        profiles=deepcopy(influences)
    else:
        profiles=[robot_response_profile()]
        profiles[0]['enabled']=False
    if enabled is not None:profiles[0]['enabled']=enabled
    EquationDynamics(model=model,influences=profiles).validate_deployment('world')
    return profiles
