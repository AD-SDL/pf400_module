"""A ResourceClient that refuses every write.

Used by a node running in simulation mode. A simulation node reads live lab state so
that its validation is meaningful, but it must never write to the resource manager:
the real lab is running experiments against the same manager, and a write from a node
that did not actually move anything would desync recorded state from physical state.

The guard is an allowlist of read methods rather than a blocklist of writes, so a
method added to ResourceClient in a future MADSci release is refused by default
instead of silently becoming writable.
"""

from typing import Any, Callable

from madsci.client.resource_client import ResourceClient


class ReadOnlyViolationError(RuntimeError):
    """Raised when a simulation node attempts to write to the resource manager."""


READ_METHODS = frozenset(
    {
        "get_resource",
        "query_resource",
        "query_resource_hierarchy",
        "query_history",
        "get_template",
        "get_template_info",
        "get_templates_by_category",
        "query_templates",
        "is_locked",
        "close",
        "unwrap",
    }
)
"""Methods that only read. Everything else on ResourceClient is refused."""


class ReadOnlyResourceClient(ResourceClient):
    """A ResourceClient whose write methods raise instead of mutating lab state.

    Reads are passed straight through to the real resource manager, so validation
    runs against true lab state. Locking methods are refused as well: a simulation
    node must not hold locks or reservations, because it is not going to release
    them by completing an action. Reservations are taken by the orchestrating agent
    against the real workcell.
    """


def _make_guard(name: str) -> Callable[..., Any]:
    """Build a replacement method that refuses the call."""

    def _guard(_self: ReadOnlyResourceClient, *_args: Any, **_kwargs: Any) -> Any:
        raise ReadOnlyViolationError(
            f"ResourceClient.{name}() was called on a simulation node. "
            "Simulation nodes are read-only: they observe live lab state but must "
            "never modify it. This is a bug in the node, not a configuration error."
        )

    _guard.__name__ = name
    _guard.__qualname__ = f"ReadOnlyResourceClient.{name}"
    _guard.__doc__ = f"Refused. {name}() writes to the resource manager."
    return _guard


def _install_guards() -> None:
    """Replace every non-read public method on the subclass with a refusal."""
    for name in dir(ResourceClient):
        if name.startswith("_") or name in READ_METHODS:
            continue
        if not callable(getattr(ResourceClient, name, None)):
            continue
        setattr(ReadOnlyResourceClient, name, _make_guard(name))


_install_guards()
