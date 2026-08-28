import csv
from pathlib import Path

REQUIRED_METADATA = (
    'start_x',
    'start_y',
    'start_yaw',
    'goal_x',
    'goal_y',
    'goal_radius',
)


def load_level(level_name, levels_dir=None):
    """Load one centerline CSV and its matching metadata TXT file."""
    if not level_name or Path(level_name).name != level_name:
        raise ValueError('El nivel debe ser un nombre simple, por ejemplo: level1')

    if levels_dir is None:
        from ament_index_python.packages import get_package_share_directory

        levels_dir = Path(get_package_share_directory('track_sim')) / 'levels'
    else:
        levels_dir = Path(levels_dir)

    csv_path = levels_dir / f'{level_name}.csv'
    metadata_path = levels_dir / f'{level_name}.txt'
    if not csv_path.is_file() or not metadata_path.is_file():
        raise FileNotFoundError(
            f'No se encontraron {csv_path.name} y {metadata_path.name} en {levels_dir}'
        )

    points = []
    with csv_path.open(newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames != ['x', 'y', 'width']:
            raise ValueError(f'{csv_path.name} debe tener el encabezado: x,y,width')
        for line_number, row in enumerate(reader, start=2):
            try:
                x = float(row['x'])
                y = float(row['y'])
                width = float(row['width'])
            except (TypeError, ValueError) as error:
                raise ValueError(f'Dato invalido en {csv_path.name}, linea {line_number}') from error
            if width <= 0.0:
                raise ValueError(f'El ancho debe ser positivo en {csv_path.name}, linea {line_number}')
            points.append((x, y, width))

    if len(points) < 2:
        raise ValueError(f'{csv_path.name} debe contener al menos dos puntos')

    metadata = {}
    with metadata_path.open(encoding='utf-8') as metadata_file:
        for line_number, raw_line in enumerate(metadata_file, start=1):
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue
            if '=' not in line:
                raise ValueError(f'Dato invalido en {metadata_path.name}, linea {line_number}')
            key, value = line.split('=', 1)
            try:
                metadata[key.strip()] = float(value.strip())
            except ValueError as error:
                raise ValueError(f'Dato invalido en {metadata_path.name}, linea {line_number}') from error

    missing = [key for key in REQUIRED_METADATA if key not in metadata]
    if missing:
        raise ValueError(f'Faltan datos en {metadata_path.name}: {", ".join(missing)}')
    if metadata['goal_radius'] <= 0.0:
        raise ValueError('goal_radius debe ser positivo')

    return {'name': level_name, 'points': points, **metadata}
