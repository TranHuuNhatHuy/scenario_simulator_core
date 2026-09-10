# Tests

`interface_manifest.reference.json` is a snapshot of
`autoware_core/common/autoware_component_interface_specs/interface_manifest.json`, used **only**
by these tests so that they run without a built `autoware_core`.

It is deliberately not used at runtime. The evaluator reads the manifest from the installed
`autoware_component_interface_specs` package and, when that is missing, reports every interface
as undeclared rather than falling back to a copy — a stale snapshot silently accepted as the
contract would defeat the purpose of joining against core's own declaration.

Refresh it whenever core's manifest changes; `test_reference_manifest_covers_declared_interfaces`
fails when an expectation names a manifest interface the snapshot does not have.
