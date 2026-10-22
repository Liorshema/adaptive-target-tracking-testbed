"""Timing, stochastic and availability contracts of the external-pose abstraction."""
from pathlib import Path
import numpy as np
import pytest
from robot_configuration.yaml_loader import load_yaml
from robot_simulation.measurements.planar_localization import PlanarLocalizationSensor


def config():
    return load_yaml(Path(__file__).parents[1]/'config/sensors/planar_localization.yaml')['localization_sensor']


def test_seeded_sensor_is_imperfect_and_preserves_acquisition_stamp():
    cfg=config();cfg['dropout_probability']=0.
    one=PlanarLocalizationSensor(cfg);two=PlanarLocalizationSensor(cfg)
    for sensor in [one,two]:sensor.observe([1,2,.3],1.)
    assert one.release(1.02)==[]
    a=one.release(1.06)[0];b=two.release(1.06)[0]
    assert a[0]==1.;np.testing.assert_array_equal(a[1],b[1]);assert not np.allclose(a[1],[1,2,.3])
    assert np.linalg.eigvalsh(a[2]).min()>0


def test_disabled_dropout_duplicate_and_reset():
    cfg=config();cfg['enabled']=False;s=PlanarLocalizationSensor(cfg);s.observe([0,0,0],0.);assert s.release(1.)==[]
    cfg=config();cfg['dropout_probability']=1.;s=PlanarLocalizationSensor(cfg);s.observe([0,0,0],0.);assert s.release(.1)==[]
    cfg=config();cfg['dropout_probability']=0.;s=PlanarLocalizationSensor(cfg);s.observe([0,0,0],2.);s.observe([0,0,0],2.)
    assert len(s.release(2.1))==1;s.observe([0,0,0],3.);s.reset();assert s.release(3.1)==[]
    s.observe([0,0,0],.1);assert len(s.release(.2))==1


def test_availability_intervals_and_finite_configuration():
    cfg=config();cfg['outages']=[[1.,2.]];s=PlanarLocalizationSensor(cfg);s.observe([0,0,0],1.5);assert not s.release(1.6)
    cfg=config();cfg['white_noise_std'][0]=0.
    with pytest.raises(ValueError):PlanarLocalizationSensor(cfg)


def test_white_noise_statistics_and_fixed_bias_are_distinct():
    cfg=config();cfg['dropout_probability']=0.;s=PlanarLocalizationSensor(cfg);values=[]
    for i in range(1500):
        stamp=i*.11;s.observe([0,0,0],stamp);values.append(s.release(stamp+.06)[0][1])
    values=np.asarray(values)
    np.testing.assert_allclose(values.std(axis=0),cfg['white_noise_std'],rtol=.1)
    np.testing.assert_allclose(values.mean(axis=0),s.bias,atol=.003)
    assert np.all(np.diag(s.covariance)>np.var(values,axis=0)*.8)
