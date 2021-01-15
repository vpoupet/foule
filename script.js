// global variables
let lastUpdate;
let canvas;
let context;
let room;
let AGENT_RADIUS = 5;
let AGENT_SPEED = .1;

// options / settings
const epsilon = .001;
let shouldDrawNeighbors = false;


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

    /**
     * Returns the distance after which a line starting from `this` moving along vector `direction` enters the circle
     * specified as parameter (center and radius).
     *
     * If the line never meets the circle, it returns Infinity. Only the half line going in the orientation of the
     * vector is considered. If the point is in the circle but moving away from its center, it is considered to be
     * "escaping" and the function returns Infinity.
     *
     * For practical reasons, if the movement direction is "almost" orthogonal to the direction towards the center of
     * the circle, the point is also considered to be escaping.
     *
     * @param {Vector} center
     * @param {number} radius
     * @param {Vector} direction (must be a unit vector)
     * @returns {number}
     */
    distanceToCircleAlongVector(center, radius, direction) {
        if (center.subtract(this).dot(direction) <= epsilon) {
            // moving away from the circle
            return Infinity;
        }
        if (this.distanceTo(center) <= radius) {
            // this is already in the circle
            return 0;
        }

        const u = center.subtract(this);
        const uNormal = u.dot(direction.orth());
        if (Math.abs(uNormal) > radius) return Infinity; // trajectory of this never touches the circle
        const uParallel = u.dot(direction);
        if (uParallel < 0) return Infinity; // this is moving away from the circle
        return uParallel - Math.sqrt(radius * radius - uNormal * uNormal);
    }

    /**
     *
     * @param {Vector} pointA start extremity of segment
     * @param {Vector} pointB end extremity of segment
     * @param {Vector} direction normal vector representing the direction of movement from `this`
     * @returns {number}
     */
    distanceToSegmentAlongVector(pointA, pointB, direction) {
        const vAB = pointB.subtract(pointA);
        const vAP = this.subtract(pointA);
        const vBP = this.subtract(pointB);

        const denominator = vAB.det(direction);
        if (denominator >= 0) {
            // direction is moving away from the segment (in the direction of the normal)
            return Infinity;
        }
        const numerator = vAP.det(vAB);
        if (direction.det(vAP) * direction.det(vBP) <= 0) {
            // trajectory of this intersects the segment [AB]
            const d = numerator / denominator;
            return d >= 0 ? d : Infinity;
        }
        return Infinity;
    }

    /**
     * Returns the angle in radians (in an interval from 0 to 2PI) between `this` and `other`
     * @param {Vector} other
     * @returns {number}
     */
    angleTo(other) {
        let angle = Math.atan2(other.y, other.x) - Math.atan2(this.y, this.x);
        return angle < 0 ? angle + 2 * Math.PI : angle;
    }
}


class ObstacleVertex extends Vector {
    constructor(x, y) {
        super(x, y);
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

    drawNeighbors(ctx) {
        ctx.beginPath();
        for (const vertex of this.neighbors) {
            ctx.moveTo(this.x, this.y);
            ctx.lineTo(vertex.x, vertex.y);
        }
        ctx.stroke();
    }
}


/**
 * Represents a (non-crossing) polygonal-shaped obstacle
 */
class Obstacle {
    constructor(vertices) {
        /**
         * Vertices of the obstacle
         * Vertices should be listed in direct order, so that when considering segments vi = [V_iV_i+1], the normal
         * vi.orth() is pointing outwards.
         *
         * @type {ObstacleVertex[]}
         */
        this.vertices = vertices.map(v => new ObstacleVertex(v.x, v.y));
    }

    /**
     * Returns a generator that enumerates the segments of the obstacle
     * (each segment is an array of two ObstacleVertex)
     *
     * @returns {Generator<ObstacleVertex[]>}
     */
    * segments() {
        for (let i = 0; i < this.vertices.length - 1; i++) {
            yield [this.vertices[i], this.vertices[i + 1]];
        }
        yield [this.vertices[this.vertices.length - 1], this.vertices[0]];
    }

    distanceFromPointAlongVector(point, direction) {
        let distance = Infinity;
        for (const [pointA, pointB] of this.segments()) {
            let vertex;
            if (point.equals(pointA)) vertex = pointA;
            if (point.equals(pointB)) vertex = pointB;
            if (vertex !== undefined) {
                // point is an extremity of the segment
                const i = this.vertices.indexOf(vertex);
                const n = this.vertices.length;
                const v1 = this.vertices[(i + n - 1) % n].subtract(vertex); // vector vertex -> previous vertex
                const v2 = this.vertices[(i + 1) % n].subtract(vertex);     // vector vertex -> next vertex
                if (v2.angleTo(direction) > v2.angleTo(v1) + epsilon) {
                    // direction is going towards the inside of the obstacle
                    return 0;
                }   // else no constraint from this segment
            } else {
                distance = Math.min(distance, point.distanceToSegmentAlongVector(pointA, pointB, direction));
            }
        }
        return distance;
    }

    containsPoint(point) {
        let nbIntersections = 0;
        for (const [pointA, pointB] of this.segments()) {

        }
        return nbIntersections % 2 === 1;
    }

    draw(ctx) {
        ctx.beginPath();
        ctx.moveTo(this.vertices[0].x, this.vertices[0].y);
        for (let i = 1; i < this.vertices.length; i++) {
            ctx.lineTo(this.vertices[i].x, this.vertices[i].y);
        }
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        // DEBUG (draw all vertices that can be reached in a straight line)
        if (shouldDrawNeighbors) {
            ctx.save();
            ctx.strokeStyle = "#82bade";
            for (const vertex of this.vertices) {
                vertex.drawNeighbors(ctx);
            }
            ctx.restore();
        }
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
        this.isLoaded = true;
        this.width = width;
        this.height = height;
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
        const dist = point1.distanceTo(point2);
        const direction = point2.subtract(point1).normalize();

        for (const obstacle of this.obstacles) {
            if (obstacle.distanceFromPointAlongVector(point1, direction) < dist - epsilon) {
                return false;
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

    update() {
        const now = Date.now();
        const deltaTime = 1000 / 60;
        lastUpdate = now;

        if (this.isRunning) {
            for (const agent of this.agents) {
                Agent.update.call(agent, deltaTime, this);
            }
        }

        context.clearRect(0, 0, canvas.width, canvas.height);
        this.draw(context);
        if (this.isLoaded) requestAnimationFrame(this.update.bind(this));
    }
}


class Agent {
    static update = Agent.prototype.updateWithSimpleDeviation;

    constructor(position, radius, speed) {
        this.position = position;
        this.radius = radius;
        this.speed = speed;
    }

    simpleUpdate(deltaTime, room) {
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

    updateWithSimpleCollision(deltaTime, room) {
        let target = room.targetFromPoint(this.position);
        if (target === undefined) {
            // no path to exit (or already reached exit)
            room.agents.delete(this);
            return;
        }

        let moveDistance = this.speed * deltaTime;
        const moveDirection = target.subtract(this.position).normalize();

        // check for collisions with other agents
        for (const otherAgent of room.agents) {
            if (otherAgent === this) continue;
            moveDistance = Math.min(moveDistance,
                this.position.distanceToCircleAlongVector(
                    otherAgent.position,
                    this.radius + otherAgent.radius,
                    moveDirection));
        }
        if (this.position.distanceTo(target) <= moveDistance) {
            this.position = target;
        } else {
            this.position = this.position.add(moveDirection.mult(moveDistance));
        }
    }

    updateWithSimpleDeviation(deltaTime, room) {
        let target = room.targetFromPoint(this.position);
        if (target === undefined) {
            // no path to exit (or already reached exit)
            room.agents.delete(this);
            return;
        }

        let moveDistance = this.speed * deltaTime;
        let moveDirection = target.subtract(this.position).normalize();

        let deviationFactor = 0;
        let deviationVector = undefined;
        for (const otherAgent of room.agents) {
            if (otherAgent === this) continue;
            if (this.position.distanceTo(otherAgent.position) >= this.radius + otherAgent.radius) continue;

            const v = otherAgent.position.subtract(this.position).normalize();
            const d = v.dot(moveDirection);
            if (d > deviationFactor) {  // this agent causes a bigger deviation
                // move orthogonally to the direction to the position of the agent
                deviationFactor = d;
                deviationVector = v.det(moveDirection) > 0 ? v.orth() : v.orth().mult(-1);
            }
        }

        if (deviationVector === undefined) {
            if (this.position.distanceTo(target) <= moveDistance) {
                this.position = target;
            } else {
                this.position = this.position.add(moveDirection.mult(moveDistance));
            }
        } else {
            this.moveAlongVector(deviationVector, moveDistance, room);
        }
    }

    moveAlongVector(direction, distance, room) {
        for (const obstacle of room.obstacles) {
            distance = Math.min(distance, obstacle.distanceFromPointAlongVector(this.position, direction));
        }
        for (const agent of room.agents) {
            if (agent === this) continue;
            distance = Math.min(distance,
                this.position.distanceToCircleAlongVector(agent.position, this.radius + agent.radius, direction));
        }
        this.position = this.position.add(direction.mult(distance));
    }

    draw(context) {
        context.beginPath();
        context.arc(this.position.x, this.position.y, this.radius, 0, 2 * Math.PI);
        context.fill();
    }
}


window.onload = function () {
    canvas = document.getElementById("canvas");
    context = canvas.getContext("2d");
    const startButton = document.getElementById("start-button");
    startButton.addEventListener("click", event => {
        room.isRunning = !room.isRunning;
        if (room.isRunning) {
            startButton.innerHTML = "Pause";
        } else {
            startButton.innerHTML = "Start";
        }
    });

    const clearButton = document.getElementById("clear-button");
    clearButton.addEventListener("click", event => {
        room.isRunning = false;
        startButton.innerHTML = "Start";
        room.agents.clear();
    });

    const addAgentButton = document.getElementById("add-one-button");
    addAgentButton.addEventListener("click", event => {
        addAgent();
    });

    const addAgentsButton = document.getElementById("add-ten-button");
    addAgentsButton.addEventListener("click", event => {
        for (let i = 0; i < 10; i++) {
            addAgent();
        }
    });

    canvas.addEventListener("click", event => {
        const rect = document.getElementById("canvas").getBoundingClientRect();
        const x = (event.clientX - rect.left);
        const y = (event.clientY - rect.top);
        room.agents.add(new Agent(new Vector(x, y), AGENT_RADIUS, AGENT_SPEED));
    });

    loadRoom(0);
}

function selectStrategy(element) {
    switch (element.value) {
        case "none":
            Agent.update = Agent.prototype.simpleUpdate;
            break;
        case "simple":
            Agent.update = Agent.prototype.updateWithSimpleCollision;
            break;
        case "deviation":
            Agent.update = Agent.prototype.updateWithSimpleDeviation;
            break;
        default:
            break;
    }
}

function toggleNeighbors() {
    const element = document.getElementById("draw-neighbors");
    shouldDrawNeighbors = element.checked;
}

function addAgent() {
    const x = Math.random() * room.width;
    const y = Math.random() * room.height;
    room.agents.add(new Agent(new Vector(x, y), AGENT_RADIUS, AGENT_SPEED));
}

function loadRoom(roomID) {
    if (room) {
        // unload current room
        room.isLoaded = false;
        room.isRunning = false;
        room.agents.clear();
    }

    room = availableRooms[roomID];
    if (room === undefined) {
        room = availableRooms[0];
    }
    room.isLoaded = true;

    document.getElementById("start-button").innerHTML = "Start";
    canvas.width = room.width;
    canvas.height = room.height;

    lastUpdate = Date.now();
    requestAnimationFrame(room.update.bind(room));
}