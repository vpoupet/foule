let lastUpdate;
let canvas;
let context;
const scale = 50;
const shiftX = 1;
const shiftY = 1;
const agentRadius = .3;
let room;



class Vector {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    add(other) {
        return new Vector(this.x + other.x, this.y + other.y);
    }

    sub(other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }

    mult(s) {
        return new Vector(this.x * s, this.y * s);
    }

    dot(other) {
        return this.x * other.x + this.y * other.y;
    }

    det(other) {
        return this.x * other.y - other.x * this.y;
    }

    squareNorm() {
        return this.x * this.x + this.y * this.y;
    }

    norm() {
        return Math.sqrt(this.squareNorm());
    }

    distanceTo(other) {
        return other.sub(this).norm();
    }

    squareDistanceTo(other) {
        return other.sub(this).squareNorm();
    }

    orth() {
        return new Vector(this.y, -this.x);
    }

    equals(other) {
        return this.x === other.x && this.y === other.y;
    }

    normalize() {
        const n = this.norm();
        if (n !== 0) {
            return new Vector(this.x / n, this.y / n);
        } else {
            return new Vector();
        }
    }
}

/**
 * Returns whether two segments intersect
 *
 * @param p1 origin of first segment
 * @param p2 end of first segment
 * @param p3 origin of second segment
 * @param p4 end of second segment
 * @returns {boolean} true if segments [p1p2] and [p3p4] intersect
 */
function segmentsIntersect(p1, p2, p3, p4) {
    const normal1 = p2.sub(p1).orth();
    const normal2 = p4.sub(p3).orth();

    if ((p3.sub(p1).dot(normal1) * p4.sub(p1).dot(normal1) <= 0) &&
        (p1.sub(p3).dot(normal2) * p2.sub(p3).dot(normal2) <= 0)) {
        // projection of [p3p4] along [p1p2] contains 0 and
        // projection of [p1p2] along [p3p4] contains 0
        if (p2.sub(p1).det(p3.sub(p1)) === 0 &&
            p2.sub(p1).det(p4.sub(p1)) === 0) {
            // all 4 points are aligned
            return Math.max(p1.x, p2.x) >= Math.min(p3.x, p4.x) &&
                Math.max(p3.x, p4.x) >= Math.min(p1.x, p2.x) &&
                Math.max(p1.y, p2.y) >= Math.min(p3.y, p4.y) &&
                Math.max(p3.y, p4.y) >= Math.min(p1.y, p2.y);
        }
        return true;
    }
    return false;
}


function getCircleContactAlongVector(p1, p2, v, d) {
    const v2 = p2.sub(p1);
    const v2Normal = Math.abs(v2.dot(v.orth()));
    if (v2Normal > d) return Infinity;
    const v2Parallel = v2.dot(v);
    if (v2Parallel < 0) return Infinity;
    return v2Parallel - Math.sqrt(d*d - v2Normal * v2Normal);
}

class ObstacleVertex extends Vector {
    constructor(x, y, obstacle) {
        super(x, y);
        this.obstacle = obstacle;
        this.neighbors = [];
        this.target = undefined;    // vector towards which to move next (ObstacleVertex or Door point)
        this.distanceToExit = Infinity;
    }

    updateDistanceToExit() {
        let didUpdate = false;
        for (const vertex of this.neighbors) {
            const d = vertex.distanceToExit + this.distanceTo(vertex);
            if (d < this.distanceToExit) {
                this.distanceToExit = d;
                this.target = vertex;
                didUpdate = true;
            }
        }
        if (didUpdate) {
            for (const vertex of this.neighbors) {
                vertex.updateDistanceToExit();
            }
        }
    }
}

/**
 * Represents a (non-crossing) polygonal-shaped obstacle
 */
class Obstacle {
    constructor(vertices, isClosed = true) {
        /**
         * List of vertices
         * @type {ObstacleVertex[]}
         */
        this.vertices = vertices.map(v => new ObstacleVertex(v.x, v.y, this));
        this.isClosed = isClosed;
    }

    * segments() {
        for (let i = 0; i < this.vertices.length - 1; i++) {
            yield [this.vertices[i], this.vertices[i + 1]];
        }
        if (this.isClosed) {
            yield [this.vertices[this.vertices.length - 1], this.vertices[0]];
        }
    }

    static rectangle(x1, y1, x2, y2) {
        return new Obstacle([
            new Vector(x1, y1),
            new Vector(x2, y1),
            new Vector(x2, y2),
            new Vector(x1, y2),
        ]);
    }

    /**
     * Returns true if a line from vertex (a vertex of the obstacle) towards a target point does not immediately move
     * inside the obstacle (only for closed obstacles)
     * @param vertex
     * @param targetPoint
     * @returns {boolean}
     */
    canGoFromTowards(vertex, targetPoint) {
        const i = this.vertices.indexOf(vertex);
        const n = this.vertices.length;
        const normal1 = vertex.sub(this.vertices[(i + n - 1) % n]).orth();
        const normal2 = this.vertices[(i + 1) % n].sub(vertex).orth();
        return targetPoint.sub(vertex).dot(normal1) >= 0 || targetPoint.sub(vertex).dot(normal2) >= 0;
    }

    draw(ctx) {
        ctx.beginPath();
        ctx.moveTo(this.vertices[0].x, this.vertices[0].y);
        for (let i = 1; i < this.vertices.length; i++) {
            ctx.lineTo(this.vertices[i].x, this.vertices[i].y);
        }
        if (this.isClosed) {
            ctx.closePath();
        }
        ctx.fill();
        ctx.stroke();
    }
}

class Door {
    constructor(p1, p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    targetFromPoint(p) {
        const vector = this.p2.sub(this.p1);
        const norm = vector.norm();
        const normalizedVector = vector.normalize();
        const l = p.sub(this.p1).dot(normalizedVector);

        if (l <= 0) return this.p1;
        else if (l >= norm) return this.p2;
        else return this.p1.add(normalizedVector.mult(l));
    }

    draw(ctx) {
        const normal = this.p2.sub(this.p1).orth().normalize();
        ctx.beginPath();
        ctx.moveTo(this.p1.x, this.p1.y);
        let p = this.p2;
        ctx.lineTo(p.x, p.y);
        p = this.p2.add(normal.mult(.5));
        ctx.lineTo(p.x, p.y);
        p = this.p1.add(normal.mult(.5));
        ctx.lineTo(p.x, p.y);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }
}

class Room {
    constructor(doors, obstacles) {
        this.doors = doors;
        this.obstacles = obstacles;
        this.agents = new Set();

        // compute neighbors for each vertex
        const vertices = [].concat(...this.obstacles.map(o => o.vertices));
        for (let i = 0; i < vertices.length; i++) {
            for (let j = i + 1; j < vertices.length; j++) {
                if (this.canConnect(vertices[i], vertices[j])) {
                    vertices[i].neighbors.push(vertices[j]);
                    vertices[j].neighbors.push(vertices[i]);
                }
            }
        }

        // Compute distance to exit for each vertex
        for (const vertex of vertices) {
            let didUpdate = false;
            for (const door of this.doors) {
                const target = door.targetFromPoint(vertex);
                if (this.canConnect(vertex, target)) {
                    const d = vertex.distanceTo(target);
                    if (d < vertex.distanceToExit) {
                        vertex.distanceToExit = d;
                        vertex.target = target;
                        didUpdate = true;
                    }
                }
            }
            if (didUpdate) {
                for (const neighbor of vertex.neighbors) {
                    neighbor.updateDistanceToExit();
                }
            }
        }
    }

    /**
     * Returns true if it is possible to connect point1 and point2 without touching any obstacle segment other
     * than the ones containing point1 and point2 (as vertices), and without passing through a closed obstacle
     * (in the case when point1 and point2 are vertices of the same closed polygon)
     *
     * @param point1
     * @param point2
     * @returns {boolean}
     */
    canConnect(point1, point2) {
        for (const obstacle of this.obstacles) {
            for (const segment of obstacle.segments()) {
                if (!segment[0].equals(point1) &&
                    !segment[1].equals(point1) &&
                    !segment[0].equals(point2) &&
                    !segment[1].equals(point2) &&
                    segmentsIntersect(point1, point2, segment[0], segment[1])) {
                    return false;
                }
                if (obstacle.isClosed) {
                    if (point1.equals(segment[0]) &&
                        !obstacle.canGoFromTowards(segment[0], point2)) return false;
                    if (point1.equals(segment[1]) &&
                        !obstacle.canGoFromTowards(segment[1], point2)) return false;
                    if (point2.equals(segment[0]) &&
                        !obstacle.canGoFromTowards(segment[0], point1)) return false;
                    if (point2.equals(segment[1]) &&
                        !obstacle.canGoFromTowards(segment[1], point1)) return false;
                }
            }
        }
        return true;
    }

    targetFromPoint(point) {
        if (point.target !== undefined) {
            return point.target;
        }

        let distance = Infinity;
        let target = undefined;

        for (const door of this.doors) {
            let t = door.targetFromPoint(point);
            if (this.canConnect(point, t)) {
                const d = point.distanceTo(t);
                if (d === 0) return undefined;
                if (d < distance) {
                    distance = d;
                    target = t;
                }
            }
        }

        for (const obstacle of this.obstacles) {
            for (const vertex of obstacle.vertices) {
                if (point.equals(vertex)) {
                    return vertex.target;
                }
                if (this.canConnect(point, vertex)) {
                    const d = point.distanceTo(vertex) + vertex.distanceToExit;
                    if (d < distance) {
                        distance = d;
                        target = vertex;
                    }
                }
            }
        }
        return target;
    }

    draw(ctx) {
        ctx.save();
        context.scale(scale, scale);
        context.translate(shiftX, shiftY);

        // draw doors
        ctx.lineWidth = .05;
        ctx.strokeStyle = "#ff0000";
        ctx.fillStyle = "#ffaaaa";
        for (const door of this.doors) {
            door.draw(ctx);
        }

        // draw obstacles
        ctx.lineWidth = .05;
        ctx.strokeStyle = "#000000";
        ctx.fillStyle = "#aaaaaa";
        for (const obstacle of this.obstacles) {
            obstacle.draw(ctx);
        }

        // draw agents
        for (const agent of this.agents) {
            ctx.fillStyle = "#0000ff";
            agent.draw(context);
        }
        ctx.restore();
    }

    drawVectors(ctx) {
        ctx.save();
        ctx.lineWidth = .02;
        ctx.strokeStyle = "#888888";
        ctx.beginPath();
        for (let x = 0; x < 20; x += .2) {
            for (let y = 0; y < 15; y += .2) {
                const p = new Vector(x, y);
                const target = this.targetFromPoint(p);
                if (target !== undefined) {
                    const pt = p.add(target.sub(p).normalize().mult(.2));
                    ctx.moveTo(x, y);
                    ctx.lineTo(pt.x, pt.y);
                }
            }
        }
        ctx.stroke();
        ctx.restore();
    }

    drawTrajectoryFromPoint(point, ctx) {
        ctx.save();
        ctx.lineWidth = .2;
        ctx.strokeStyle = "#00ff00";
        ctx.beginPath();
        ctx.moveTo(point.x, point.y);
        while (true) {
            point = this.targetFromPoint(point);
            if (point === undefined) break;
            ctx.lineTo(point.x, point.y);
        }
        ctx.stroke();
        ctx.restore();
    }

    update() {
        const now = Date.now();
        const deltaTime = now - lastUpdate;
        lastUpdate = now;

        for (const agent of this.agents) {
            agent.update(deltaTime, this);
        }
        context.clearRect(0, 0, canvas.width, canvas.height);
        this.draw(context);
        requestAnimationFrame(this.update.bind(this));

    }
}


class Agent {
    constructor(position, speed) {
        this.position = position;
        this.speed = speed;
    }

    update(deltaTime, room) {
        let remainingMovement = this.speed * deltaTime;
        while (remainingMovement > 0) {
            let target = room.targetFromPoint(this.position);
            if (target === undefined) { // no path to exit (or already reached exit)
                room.agents.delete(this);
                return;
            }

            let moveDistance = Math.min(remainingMovement, this.position.distanceTo(target));
            remainingMovement -= moveDistance;
            const v = target.sub(this.position).normalize();

            this.position = this.position.add(v.mult(moveDistance));
        }
    }

    draw(context) {
        context.beginPath();
        context.arc(this.position.x, this.position.y, agentRadius, 0, 2*Math.PI);
        context.fill();
    }
}


class SimpleCollisionAgent extends Agent {
    update(deltaTime, room) {
        let remainingMovement = this.speed * deltaTime;
        while (remainingMovement > 0) {
            let target = room.targetFromPoint(this.position);
            if (target === undefined) { // no path to exit (or already reached exit)
                room.agents.delete(this);
                return;
            }

            let moveDistance = Math.min(remainingMovement, this.position.distanceTo(target));
            remainingMovement -= moveDistance;

            const v = target.sub(this.position).normalize();
            let squareNeighborhood = moveDistance + 2 * agentRadius;
            squareNeighborhood *= squareNeighborhood;

            // check for collisions with other agents
            for (const otherAgent of room.agents) {
                if (otherAgent === this ||
                    this.position.squareDistanceTo(otherAgent.position) > squareNeighborhood) {
                    // no collision
                    continue;
                }
                const u2 = otherAgent.position.sub(this.position);
                const u2Normal = Math.abs(u2.dot(v.orth()));
                if (u2Normal > 2 * agentRadius) continue;
                const u2Parallel = u2.dot(v);
                if (u2Parallel < 0) continue;
                const collisionDistance = u2Parallel - Math.sqrt(4 * agentRadius * agentRadius - u2Normal * u2Normal);
                if (collisionDistance < moveDistance) {
                    moveDistance = collisionDistance;
                    remainingMovement = 0;
                }
            }
            this.position = this.position.add(v.mult(moveDistance));
        }
    }
}


window.onload = function () {
    canvas = document.getElementById("canvas");
    canvas.width = 1000;
    canvas.height = 750;
    context = canvas.getContext("2d");

    canvas.addEventListener("click", event => {
        let collisionType;
        const collisionSelect = document.getElementsByName('collision-select');
        for (const select of collisionSelect) {
            if (select.checked) {
                collisionType = select.value;
                break;
            }
        }

        const rect = document.getElementById("canvas").getBoundingClientRect();
        const x = (event.clientX - rect.left) / scale - shiftX;
        const y = (event.clientY - rect.top) / scale - shiftY;
        switch(collisionType) {
            case "none":
                room.agents.add(new Agent(new Vector(x, y), .001));
                break;
            case "simple":
                room.agents.add(new SimpleCollisionAgent(new Vector(x, y), .001));
                break;
            default:
                break;
        }
    });

    room = new Room(
        [
            new Door(new Vector(0, 7), new Vector(0, 5)),
            new Door(new Vector(9, 13), new Vector(7, 13)),
        ],
        [
            Obstacle.rectangle(2, 2, 4, 5),
            Obstacle.rectangle(3, 7, 6, 10),
            Obstacle.rectangle(6, 3, 9, 5),
            Obstacle.rectangle(6, 1, 9, 2),
            Obstacle.rectangle(11, 2, 14, 6),
            Obstacle.rectangle(9, 7, 11, 9),
            Obstacle.rectangle(12, 7, 14, 9),
            Obstacle.rectangle(10, 10, 13, 11),
            new Obstacle([
                new Vector(-.5, 5),
                new Vector(-.5, -.5),
                new Vector(16.5, -.5),
                new Vector(16.5, 13.5),
                new Vector(9, 13.5),
                new Vector(9, 13),
                new Vector(16, 13),
                new Vector(16, 0),
                new Vector(0, 0),
                new Vector(0, 5),
            ]),
            new Obstacle([
                new Vector(0, 7),
                new Vector(0, 13),
                new Vector(7, 13),
                new Vector(7, 13.5),
                new Vector(-.5, 13.5),
                new Vector(-.5, 7),
            ]),
        ],
    );

    lastUpdate = Date.now();
    requestAnimationFrame(room.update.bind(room));
}