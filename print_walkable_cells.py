import csv

walkable_types = [
    '0', '.', 'i', 'c', 'd',
    'le', 'ln', 'lw', 'ls', 'lne', 'les', 'lnw', 'lse',
    'je', 'jn', 'jw', 'js', 'jne', 'jes', 'jnw', 'jse', 'jws', 'jen'
]

grid = []
with open('sample_warehouse.csv') as f:
    reader = csv.reader(f)
    grid = [row for row in reader]

print('Walkable cells:')
for y, row in enumerate(grid):
    for x, cell in enumerate(row):
        if cell.strip() in walkable_types:
            print(f'({x},{y}): {cell.strip()}') 