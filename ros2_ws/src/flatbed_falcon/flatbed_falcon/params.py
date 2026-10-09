"""Declare a dataclass's fields as ROS parameters and read them back."""
import dataclasses


def declare_dataclass(node, obj, prefix=''):
    """Every field of `obj` becomes the ROS parameter `prefix + name` with the field's
    value as default; returns a copy of `obj` with the values ROS has (from the YAML)."""
    values = {}
    for f in dataclasses.fields(obj):
        default = getattr(obj, f.name)
        values[f.name] = node.declare_parameter(prefix + f.name, default).value
    return dataclasses.replace(obj, **values)
