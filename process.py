import sys
import re
import bs4


def vertices_from_path(d, x0=0, y0=0, scale=1):
    vertices = []
    x, y = 0, 0
    for instruction in re.findall('[a-zA-Z][^a-zA-Z]*', d):
        command = instruction[0]
        if command in 'mMlLvVhH':
            arguments = tuple(int(x) for x in re.split('[ ,]+', instruction[1:]))
            if command in 'ML':
                x, y = arguments
                x = (x - x0) * scale
                y = (y - y0) * scale
            elif command in 'ml':
                x, y = x + arguments[0] * scale, y + arguments[1] * scale
            elif command == 'V':
                y = (arguments[0] - y0) * scale
            elif command == 'v':
                y += arguments[0] * scale
            elif command == 'H':
                x = (arguments[0] - x0) * scale
            elif command == 'h':
                x += arguments[0] * scale
            vertices.append((x, y))
    if vertices[0] == vertices[-1]:
        vertices.pop()
    return vertices[::-1]


def vertices_from_rect(x, y, width, height, x0=0, y0=0, scale=1):
    x = (x - x0) * scale
    y = (y - y0) * scale
    width *= scale
    height *= scale
    return [(x, y), (x, y + height), (x + width, y + height), (x + width, y)]


def make_room(input_file, output_file, scale):
    soup = bs4.BeautifulSoup(open(input_file), 'lxml')
    x0, y0, width, height = (int(x) for x in soup.find('svg')['viewbox'].split(' '))

    obstacles = []

    g = soup.find(id='Obstacles')
    for el in g:
        if el.name == 'path':
            obstacles.append(vertices_from_path(el['d'], x0, y0, scale))
        elif el.name == 'rect':
            obstacles.append(vertices_from_rect(int(el['x']), int(el['y']), int(el['width']), int(el['height']), x0, y0, scale))

    with open(output_file, 'w') as f_out:
        f_out.write('if (typeof availableRooms === "undefined") availableRooms = [];\n')
        f_out.write(f'availableRooms.push(new Room({width * scale}, {height * scale}, [\n')
        for ob in obstacles:
            f_out.write('    new Obstacle([')
            f_out.write(', '.join(f'new Vector({x}, {y})' for x, y in ob))
            f_out.write(']),\n')
        f_out.write(']));\n')


if __name__ == '__main__':
    scale = 1.
    for arg in sys.argv:
        if arg.startswith("scale="):
            scale = float(arg[6:])
            sys.argv.remove(arg)
            break

    if len(sys.argv) == 2:
        output_file = sys.argv[1].replace(".svg", ".js")
        if output_file != sys.argv[1]:
            sys.argv.append(output_file)

    if len(sys.argv) < 3:
        print('Not enough arguments.')
        print(f'{sys.argv[0]} <input.svg> <output.js>')
        exit()

    make_room(sys.argv[1], sys.argv[2], scale)
