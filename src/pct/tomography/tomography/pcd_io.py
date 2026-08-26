"""Small NumPy-only PCD reader suitable for ARM64 deployments.

PCL's normal ``binary`` and ``ascii`` PCD encodings are supported. Keeping
this reader local avoids depending on Open3D, whose Python wheels are not
consistently available for Ubuntu 20.04/aarch64.
"""

import numpy as np


_PCD_TYPES = {
    ("F", 4): "<f4", ("F", 8): "<f8",
    ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
    ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
}


def read_point_cloud_xyz(path):
    """Read x/y/z fields from an ASCII or binary PCD into float32."""
    with open(path, "rb") as stream:
        fields = sizes = types = counts = None
        width = points = None
        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise ValueError("unexpected end of PCD header")
            line = raw_line.decode("ascii", errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, raw_value = line.partition(" ")
            key = key.upper()
            values = raw_value.split()
            if key == "FIELDS":
                fields = values
            elif key == "SIZE":
                sizes = [int(value) for value in values]
            elif key == "TYPE":
                types = [value.upper() for value in values]
            elif key == "COUNT":
                counts = [int(value) for value in values]
            elif key == "WIDTH":
                width = int(values[0])
            elif key == "POINTS":
                points = int(values[0])
            elif key == "DATA":
                encoding = values[0].lower()
                break

        if fields is None or sizes is None or types is None:
            raise ValueError("PCD header is missing FIELDS, SIZE, or TYPE")
        if not {"x", "y", "z"}.issubset(fields):
            raise ValueError("PCD must contain x, y, and z fields")
        if counts is None:
            counts = [1] * len(fields)
        point_count = points if points is not None else (width or 0)

        if encoding == "ascii":
            columns = {}
            offset = 0
            for name, count in zip(fields, counts):
                columns[name] = offset
                offset += count
            xyz = []
            for raw_line in stream:
                values = raw_line.decode("ascii", errors="replace").split()
                if values:
                    xyz.append((float(values[columns["x"]]),
                                float(values[columns["y"]]),
                                float(values[columns["z"]])))
            result = np.asarray(xyz, dtype=np.float32)
        elif encoding == "binary":
            dtype_fields = []
            used_names = set()
            for name, size, field_type, count in zip(fields, sizes, types, counts):
                scalar_type = _PCD_TYPES.get((field_type, size))
                if scalar_type is None:
                    raise ValueError("unsupported PCD field type {}{}".format(
                        field_type, size))
                unique_name = name
                suffix = 1
                while unique_name in used_names:
                    unique_name = "{}_{}".format(name, suffix)
                    suffix += 1
                used_names.add(unique_name)
                dtype_fields.append((unique_name, scalar_type) if count == 1
                                    else (unique_name, scalar_type, (count,)))
            dtype = np.dtype(dtype_fields)
            expected_bytes = point_count * dtype.itemsize
            payload = stream.read(expected_bytes)
            if len(payload) != expected_bytes:
                raise ValueError("PCD binary payload is shorter than its header")
            records = np.frombuffer(payload, dtype=dtype, count=point_count)
            result = np.stack((records["x"], records["y"], records["z"]), axis=1)
        elif encoding == "binary_compressed":
            raise ValueError(
                "binary_compressed PCD is unsupported on the lightweight ARM64 "
                "path; save the map as PCL DATA binary or ascii")
        else:
            raise ValueError("unsupported PCD DATA encoding: {}".format(encoding))

    result = np.asarray(result, dtype=np.float32)
    if result.ndim != 2 or result.shape[1] != 3 or result.shape[0] == 0:
        raise ValueError("PCD contains no XYZ points")
    result = result[np.isfinite(result).all(axis=1)]
    if result.shape[0] == 0:
        raise ValueError("PCD contains no finite XYZ points")
    return np.ascontiguousarray(result, dtype=np.float32)

