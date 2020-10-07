let lastUpdate;
let canvas;
let context;
let room;
const agentRadius = 15;
const epsilon = .001;


class Vector {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    /**
     * Vector addition
     *
     * @param {Vector} other
     * @returns {Vector}
     */
    add(other) {
        return new Vector(this.x + other.x, this.y + other.y);
    }

    /**
     * Vector subtraction
     *
     * @param {Vector} other
     * @returns {Vector}
     */
    subtract(other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }

    /**
     * Scalar multiplication
     *
     * @param {number} s
     * @returns {Vector}
     */
    mult(s) {
        return new Vector(this.x * s, this.y * s);
    }

    /**
     * Dot product
     *
     * @param {Vector} other
     * @returns {number}
     */
    dot(other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Returns the determinant of the matrix
     * |xa xb|
     * |ya yb|
     * where (xa, ya) is `this` and (xb, yb) is `other`
     * (this is also the z-coordinate of the cross product of the two vectors)
     *
     * @param {Vector} other
     * @returns {number}
     */
    det(other) {
        return this.x * other.y - other.x * this.y;
    }

    /**
     * The square of the norm of the vector
     *
     * @returns {number}
     */
    squareNorm() {
        return this.x * this.x + this.y * this.y;
    }

    /**
     * The norm of the Vector
     *
     * @returns {number}
     */
    norm() {
        return Math.sqrt(this.squareNorm());
    }

    /**
     * Distance from `this` to `other` (norm of the difference)
     *
     * @param {Vector} other
     * @returns {number}
     */
    distanceTo(other) {
        return other.subtract(this).norm();
    }

    /**
     * Square of the distance from `this` to `other`
     *
     * @param {Vector} other
     * @returns {number}
     */
    squareDistanceTo(other) {
        return other.subtract(this).squareNorm();
    }

    /**
     * Returns the vector orthogonal to `this`, of same length and pointing to the right of `this`
     * (it points to the left in a traditional coordinates system but to the right in an "SVG"-compatible coordinates
     * system, with x pointing right and y pointing down)
     *
     * @returns {Vector}
     */
    orth() {
        return new Vector(-this.y, this.x);
    }

    /**
     * Returns true if `this` and `other` have same coordinates
     *
     * @param {Vector} other
     * @returns {boolean}
     */
    equals(other) {
        return this.x === other.x && this.y === other.y;
    }

    /**
     * Returns a normalized vector (length 1) of same orientation as `this` is `this` is not of length 0.
     * If `this` is the zero vector, it returns a copy of `this`.
     *
     * @returns {Vector}
     */
    normalize() {
        const n = this.norm();
        if (n !== 0) {
            return new Vector(this.x / n, this.y / n);
        } else {
            return new Vector();
        }
    }

    distanceToCircleAlongVector(center, radius, direction) {
        if (center.subtract(this).dot(direction) <= 0) {
            // moving away from the circle
            return Infinity;
        }
        if (this.distanceTo(center) <= radius) {
            // this is already in the circle
            return 0;
        }

        const u = center.subtract(this);
        const uNormal = u.dot(direction.orth());
        if (uNormal > radius) return Infinity; // trajectory of this never touches the circle
        const uParallel = u.dot(direction);
        if (uParallel < 0) return Infinity; // this is moving away from the circle
        return uParallel - Math.sqrt(radius * radius - uNormal * uNormal);
    }

    distanceToSegmentAlongVector(pointA, pointB, direction) {
        const numerator = pointB.subtract(pointA).det(this.subtract(pointA));
        const denominator = direction.det(pointB.subtract(pointA));
        if (numerator === 0) {
            // pointA, pointB and this are aligned
            if (Math.min(pointA.x, pointB.x) <= this.x &&
                this.x <= Math.max(pointA.x, pointB.x) &&
                Math.min(pointA.y, pointB.y) <= this.y &&
                this.y <= Math.max(pointA.y, pointB.y)) {
                // this is already on the segment [AB]
                return 0;
            }
            if (denominator === 0) {
                // direction is also colinear to the line connecting the points
                const dim = Math.abs(direction.x) >= Math.abs(direction.y) ? 'x' : 'y';
                const closestPoint = direction[dim] > 0 ?
                    Math.min(pointA[dim], pointB[dim]) :
                    Math.max(pointA[dim], pointB[dim]);
                const d = (Math.min(pointA[dim], pointB[dim]) - this[dim]) / direction[dim];
                return d > 0 ? d : Infinity;
            } else {
                return Infinity;
            }
        }
        if (pointA.subtract(this).det(direction) * pointB.subtract(this).det(direction) <= 0) {
            // trajectory of this intersects the segment [AB]
            const d = numerator / denominator;
            return d >= 0 ? d : Infinity;
        }
        return Infinity;
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
    const v12 = p2.subtract(p1);
    const v34 = p4.subtract(p3);

    if ((p3.subtract(p1).det(v12) * p4.subtract(p1).det(v12) <= 0) &&
        (p1.subtract(p3).det(v34) * p2.subtract(p3).det(v34) <= 0)) {
        // projection of [p3p4] along [p1p2] contains 0 and
        // projection of [p1p2] along [p3p4] contains 0
        if (p2.subtract(p1).det(p3.subtract(p1)) === 0 &&
            p2.subtract(p1).det(p4.subtract(p1)) === 0) {
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
    const v2 = p2.subtract(p1);
    const v2Normal = Math.abs(v2.det(v));
    if (v2Normal > d) return Infinity;
    const v2Parallel = v2.dot(v);
    if (v2Parallel < 0) return Infinity;
    return v2Parallel - Math.sqrt(d * d - v2Normal * v2Normal);
}

class ObstacleVertex extends Vector {
    constructor(x, y, obstacle) {
        super(x, y);
        this.obstacle = obstacle;
        this.neighbors = [];
        this.target = undefined;    // vector towards which to move next (ObstacleVertex or Exit point)
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
        const normal1 = vertex.subtract(this.vertices[(i + n - 1) % n]).orth();
        const normal2 = this.vertices[(i + 1) % n].subtract(vertex).orth();
        return targetPoint.subtract(vertex).dot(normal1) >= 0 || targetPoint.subtract(vertex).dot(normal2) >= 0;
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

class Exit {
    constructor(p1, p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    targetFromPoint(p) {
        const vector = this.p2.subtract(this.p1);
        const norm = vector.norm();
        const normalizedVector = vector.normalize();
        const l = p.subtract(this.p1).dot(normalizedVector);

        if (l <= 0) return this.p1;
        else if (l >= norm) return this.p2;
        else return this.p1.add(normalizedVector.mult(l));
    }
}

class Room {
    constructor(width, height, obstacles) {
        this.obstacles = obstacles;
        this.exits = [
            new Exit(new Vector(0, 0), new Vector(width, 0)),
            new Exit(new Vector(0, 0), new Vector(0, height)),
            new Exit(new Vector(0, height), new Vector(width, height)),
            new Exit(new Vector(width, 0), new Vector(width, height)),
        ];
        this.agents = new Set();
        this.isRunning = false;

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
            for (const exit of this.exits) {
                const target = exit.targetFromPoint(vertex);
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

        for (const exit of this.exits) {
            let t = exit.targetFromPoint(point);
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

        // draw obstacles
        ctx.lineWidth = 1;
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

    drawTrajectoryFromPoint(point, ctx) {
        ctx.save();
        ctx.lineWidth = 1;
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

        if (this.isRunning) {
            for (const agent of this.agents) {
                Agent.update.call(agent, deltaTime, this);
            }
        }

        context.clearRect(0, 0, canvas.width, canvas.height);
        this.draw(context);
        requestAnimationFrame(this.update.bind(this));
    }
}


class Agent {
    static update = Agent.simpleUpdate;

    constructor(position, speed) {
        this.position = position;
        this.speed = speed;
    }

    static simpleUpdate(deltaTime, room) {
        let target = room.targetFromPoint(this.position);
        if (target === undefined) {
            // no path to exit (or already reached exit)
            room.agents.delete(this);
            return;
        }

        const moveDistance = this.speed * deltaTime;
        if (this.position.distanceTo(target) <= moveDistance) {
            this.position = target;
        } else {
            const moveDirection = target.subtract(this.position).normalize();
            this.position = this.position.add(moveDirection.mult(moveDistance));
        }
    }

    static updateWithSimpleCollision(deltaTime, room) {
        let target = room.targetFromPoint(this.position);
        if (target === undefined) {
            // no path to exit (or already reached exit)
            room.agents.delete(this);
            return;
        }

        let moveDistance = this.speed * deltaTime;
        const moveDirection = target.subtract(this.position).normalize();
        let squareNeighborhood = moveDistance + 2 * agentRadius;
        squareNeighborhood *= squareNeighborhood;

        // check for collisions with other agents
        for (const otherAgent of room.agents) {
            if (otherAgent === this ||
                this.position.squareDistanceTo(otherAgent.position) > squareNeighborhood) {
                // no possible collision
                continue;
            }
            const u2 = otherAgent.position.subtract(this.position);
            const u2Normal = Math.abs(u2.dot(moveDirection.orth()));
            if (u2Normal > 2 * agentRadius) continue;
            const u2Parallel = u2.dot(moveDirection);
            if (u2Parallel < 0) continue;
            const collisionDistance = u2Parallel - Math.sqrt(4 * agentRadius * agentRadius - u2Normal * u2Normal);
            if (collisionDistance < moveDistance) {
                moveDistance = collisionDistance;
            }
        }
        if (this.position.distanceTo(target) <= moveDistance) {
            this.position = target;
        } else {
            this.position = this.position.add(moveDirection.mult(moveDistance));
        }
    }

    static updateWithSimpleDeviation(deltaTime, room) {
        let target = room.targetFromPoint(this.position);
        if (target === undefined) {
            // no path to exit (or already reached exit)
            room.agents.delete(this);
            return;
        }
        let moveDistance = this.speed * deltaTime;
        let moveDirection = target.subtract(this.position).normalize();

        let contactAgents = [];
        for (const otherAgent of room.agents) {
            if (this.position.distanceTo(otherAgent.position) < 2 * agentRadius + epsilon) {
                contactAgents.push(otherAgent);
            }
        }

        let maxDeviationFactor = 0;
        let maxDeviation = moveDirection;
        for (const otherAgent of contactAgents) {
            let directionFromOther = this.position.sub(otherAgent.position).normalize()
            let factor = directionFromOther.dot(moveDirection);
            if (factor < maxDeviationFactor) {
                maxDeviationFactor = factor;
                maxDeviation = directionFromOther.orth();
                if (maxDeviation.dot(moveDirection) < 0) {
                    maxDeviation = maxDeviation.mult(-1);
                }
            }
        }
        moveDirection = maxDeviation;

        let squareNeighborhood = moveDistance + 2 * agentRadius;
        squareNeighborhood *= squareNeighborhood;

        // check for collisions with other agents
        for (const otherAgent of room.agents) {
            if (otherAgent === this ||
                contactAgents.includes(otherAgent) ||
                this.position.squareDistanceTo(otherAgent.position) > squareNeighborhood) {
                // no possible collision
                continue;
            }
            const u2 = otherAgent.position.subtract(this.position);
            const u2Normal = Math.abs(u2.dot(moveDirection.orth()));
            if (u2Normal > 2 * agentRadius) continue;
            const u2Parallel = u2.dot(moveDirection);
            if (u2Parallel < 0) continue;
            const collisionDistance = u2Parallel - Math.sqrt(4 * agentRadius * agentRadius - u2Normal * u2Normal);
            if (collisionDistance < moveDistance) {
                moveDistance = collisionDistance;
            }
        }
        if (this.position.distanceTo(target) <= moveDistance) {
            this.position = target;
        } else {
            this.position = this.position.add(moveDirection.mult(moveDistance));
        }
    }

    moveTo(target) {

    }

    draw(context) {
        context.beginPath();
        context.arc(this.position.x, this.position.y, agentRadius, 0, 2 * Math.PI);
        context.fill();
    }
}


window.onload = function () {
    canvas = document.getElementById("canvas");
    canvas.width = 1200;
    canvas.height = 800;
    context = canvas.getContext("2d");

    let startButton = document.getElementById("start-button");
    startButton.addEventListener("click", event => {
        room.isRunning = !room.isRunning;
        if (room.isRunning) {
            startButton.innerHTML = "Pause";
        } else {
            startButton.innerHTML = "Start";
        }
    });

    let clearButton = document.getElementById("clear-button");
    clearButton.addEventListener("click", event => {
        room.isRunning = false;
        startButton.innerHTML = "Start";
        room.agents.clear();
    });

    canvas.addEventListener("click", event => {
        const rect = document.getElementById("canvas").getBoundingClientRect();
        const x = (event.clientX - rect.left);
        const y = (event.clientY - rect.top);
        room.agents.add(new Agent(new Vector(x, y), .1));
    });

    lastUpdate = Date.now();
    requestAnimationFrame(room.update.bind(room));
}

function selectStrategy(element) {
    switch (element.value) {
        case "none":
            Agent.update = Agent.simpleUpdate;
            break;
        case "simple":
            Agent.update = Agent.updateWithSimpleCollision;
            break;
        default:
            break;
    }
}