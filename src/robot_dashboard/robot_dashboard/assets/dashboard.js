/* Keep secondary parameter editors exclusive without touching any draft values. */
document.addEventListener('toggle', function (event) {
    const opened = event.target;
    if (!opened.open || !opened.matches('.component-parameters, .other-models')) return;
    const workspace = opened.closest('#component-cards');
    if (!workspace) return;
    workspace.querySelectorAll('.component-parameters[open], .other-models[open]').forEach(function (other) {
        if (other !== opened && !other.contains(opened)) other.open = false;
    });
}, true);
