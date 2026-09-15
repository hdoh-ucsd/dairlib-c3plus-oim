"""JSON conversion and strict parsing, without experiment policy."""
import json
import math

def _reject_constant(value):
    raise ValueError(f"Nonfinite JSON value: {value}")


def _json(text):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result
    return json.loads(text, parse_constant=_reject_constant, object_pairs_hook=unique)


def _json_values(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, dict):
        return {key: _json_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_values(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    return None if isinstance(value, float) and not math.isfinite(value) else value
