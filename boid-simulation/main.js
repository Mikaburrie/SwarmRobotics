
Math.wrapToPi = angle => {
    return angle - 2*Math.PI*Math.round(angle/Math.PI/2)
}

class Vector2 {

    static fromAngle(angle) {
        return new Vector2(Math.cos(angle), Math.sin(angle))
    }

    constructor(x = 0, y = 0) {
        this.set(x, y)
    }

    get mag() {
        return Math.sqrt(this.x*this.x + this.y*this.y)
    }

    get angle() {
        return Math.atan2(this.y, this.x)
    }

    set(x, y) {
        if (x instanceof Vector2) {
            this.x = x.x
            this.y = x.y
            return this
        }

        if (typeof x !== "number") throw new Error(`Invalid argument type: Got ${x}, expected number`)
        if (typeof y !== "number") throw new Error(`Invalid argument type: Got ${y}, expected number`)

        this.x = x
        this.y = y
        return this
    }

    add(val) {
        switch (val.constructor.name) {
            case "Vector2": return new Vector2(this.x + val.x, this.y + val.y)
            case "Number": return new Vector2(this.x + val, this.y + val)
            default: throw new Error(`Invalid argument type: Got ${val}, expected Vector2 or Number`)
        }
    }

    sub(val) {
        switch (val.constructor.name) {
            case "Vector2": return new Vector2(this.x - val.x, this.y - val.y)
            case "Number": return new Vector2(this.x - val, this.y - val)
            default: throw new Error(`Invalid argument type: Got ${val}, expected Vector2 or Number`)
        }
    }

    scale(val) {
        switch (val.constructor.name) {
            case "Vector2": return new Vector2(this.x*val.x, this.y*val.y)
            case "Number": return new Vector2(this.x*val, this.y*val)
            default: throw new Error(`Invalid argument type: Got ${val}, expected Vector2 or Number`)
        }
    }

    dot(val) {
        if (!(val instanceof Vector2)) throw new Error(`Invalid argument type: Got ${val}, expected Vector2`)
        return this.x*val.x + this.y*val.y
    }

    cross(val) {
        if (!(val instanceof Vector2)) throw new Error(`Invalid argument type: Got ${val}, expected Vector2`)
        return this.x*val.y - this.y*val.x
    }

    eval(op) {
        if (typeof op !== "function") throw new Error(`Invalid argument type: Got ${val}, expected unary or binary function`)
        if (op.length === 0) throw new Error(`Invalid operator parameter count: Got 0, expected 1 or 2`)
        if (op.length === 1) return new Vector2(op(this.x), op(this.y))
        return op(this.x, this.y)
    }
}


class Boid {
    
    // Vision characteristics
    static #fov = 60*(Math.PI/180)
    static #range = 60
    
    // Dimensions of boid
    static #size = 5
    static #radius = 0.6*this.#size
    
    // Wheel parameters
    static #wheelWidth = 1
    static #wheelDiameter = 2.5
    static #wheelBase = this.#size + this.#wheelWidth
    
    // Motor parameters
    static #motorTimeConstant = 3
    static #motorMaxSpeed = 20
    
    // Encoder parameters
    static #encoderTicksPerRotation = 20
    static #encoderDistancePerTick = Math.PI*this.#wheelDiameter/this.#encoderTicksPerRotation
    
    // Paths for drawing
    static #pathRobot
    static #pathBound
    static #pathArea

    // Getters for static private variables
    static get fov() { return this.#fov }
    static get range() { return this.#range }
    static get size() { return this.#size }
    static get wheelDiameter() { return this.#wheelDiameter }
    static get wheelBase() { return this.#wheelBase }
    static get encoderTicksPerRevolution() { return this.#encoderTicksPerRotation }
    static get encoderDistancePerTick() { return this.#encoderDistancePerTick }

    // Sets vision characteristics of boid
    static setDetectionArea(fov, range) {
        if (typeof fov !== "number") throw new Error(`Invalid argument type: Got ${typeof fov}, expected number`)
        if (typeof range !== "number") throw new Error(`Invalid argument type: Got ${typeof range}, expected number`)
        this.#fov = Math.abs(fov)
        this.#range = Math.abs(range)
        this.#calculatePathArea()
    }

    // Calculates path for detection area
    static #calculatePathArea() {
        const fov = this.#fov
        const range = this.#range
        this.#pathArea = new Path2D()
        if (fov < 2*Math.PI) {
            this.#pathArea.moveTo(range*Math.cos(fov/2), range*Math.sin(fov/2))
            this.#pathArea.lineTo(0, 0)
            this.#pathArea.lineTo(range*Math.cos(fov/2), -range*Math.sin(fov/2))
        }
        this.#pathArea.arc(0, 0, range, -fov/2, fov/2)
    }

    // Calculates paths for body + wheels + center line and bounding circle
    static #calculatePathRobot() {
        const size = this.#size
        const wheelWidth = this.#wheelWidth
        const wheelDiameter = this.#wheelDiameter
        this.#pathRobot = new Path2D()
        this.#pathRobot.rect(-size/2, -size/2, size, size)
        this.#pathRobot.rect(-wheelDiameter/2, -wheelWidth - size/2, wheelDiameter, wheelWidth)
        this.#pathRobot.rect(-wheelDiameter/2, size/2, wheelDiameter, wheelWidth)
        this.#pathRobot.moveTo(0, 0)
        this.#pathRobot.lineTo(size/2, 0)

        this.#pathBound = new Path2D()
        this.#pathBound.arc(0, 0, this.#radius, 0, 2*Math.PI)
    }

    // Calculates default paths
    static {
        this.#calculatePathArea()
        this.#calculatePathRobot()
    }

    // Private properties
    #pos
    #theta
    #dir
    #actualVelocity
    #encoder
    #rule

    // Constructs boid from pose and movement rule
    constructor(x, y, theta, rule) {
        this.#pos = new Vector2(x, y)
        this.#theta = theta
        this.#dir = Vector2.fromAngle(theta)
        this.outputPower = new Vector2
        this.#actualVelocity = new Vector2
        this.#encoder = new Vector2
        this.rule = rule
    }

    get encoder() { return this.#encoder.eval(Math.floor) }
    get rule() { return this.#rule }

    // Sets rule based on type
    set rule(rule) {
        let newRule = rule

        if (typeof rule === "function") {
            if (rule.length !== 0) throw new Error(`Invalid rule: Got ${rule}, expected 0 parameter constructor`)
            newRule = new rule()
        } else if (typeof rule !== "object") throw new Error(`Invalid argument type: Got ${val}, expected constructor function or object`)
        
        if (newRule.updateMovement !== undefined && !(typeof newRule.updateMovement === "function" && newRule.updateMovement?.length === 2)) throw new Error(`Invalid rule ${rule}: Could not find updateMovement function with 2 parameters`)
        this.#rule = newRule
    }

    // Applies movement rule
    update(boids, dt) {
        let detectedBoids = this.#getDetectedBoids(boids)
        this.#rule.updateMovement.bind(this)(detectedBoids, dt)
    }

    #getDetectedBoids(boids) {
        // Find unit vectors for edges of view range
        const fov = Boid.#fov
        const range = Boid.#range
        const radius = Boid.#radius
        let t1 = this.#theta + fov/2
        let t2 = this.#theta - fov/2
        let l1 = Vector2.fromAngle(t1)
        let l2 = Vector2.fromAngle(t2)
        
        // Finds boids in detection range
        let detected = []
        for (const boid of boids) {
            if (boid === this) continue

            let dp = boid.#pos.sub(this.#pos)
            let inl1 = radius/2 < dp.cross(l1)
            let inl2 = radius/2 < -dp.cross(l2)
            let inView = (fov > Math.PI) ? (fov >= 2*Math.PI) ? true : inl1 || inl2 : inl1 && inl2
            if (inView && dp.dot(dp) < range*range) {
                dp.set(this.#dir.dot(dp), this.#dir.cross(dp))
                detected.push({
                    rule: boid.#rule.constructor.name,
                    dist: dp.x,
                    angle: dp.angle,
                    heading: -Math.wrapToPi(boid.#theta - this.#theta + Math.PI)
                })
            }
        }

        return detected
    }

    // Moves boid across time
    integrate(dt) {
        // Limit output velocity
        this.outputPower.x = Math.min(Math.max(-1, this.outputPower.x), 1)
        this.outputPower.y = Math.min(Math.max(-1, this.outputPower.y), 1)

        // Update motor velocities
        let diff = this.outputPower.scale(Boid.#motorMaxSpeed).sub(this.#actualVelocity)
        this.#actualVelocity = this.#actualVelocity.add(diff.scale(Boid.#motorTimeConstant*dt))
        
        // Update encoder values
        let dist = this.#actualVelocity.scale(dt)
        this.#encoder = this.#encoder.add(dist.scale(1/Boid.#encoderDistancePerTick))

        // Calculate angle of travel
        let alpha = (dist.y - dist.x)/Boid.#wheelBase
        
        if (alpha === 0) {
            // Drive straight if alpha is zero
            this.#pos = this.#pos.add(this.#dir.scale(dist))
        } else {
            // Calculate turn if not straight
            let radius = 0.5*(dist.y + dist.x)/alpha
            let deltaP = new Vector2(Math.sin(alpha), 1 - Math.cos(alpha)).scale(radius)
            let rotationVec = new Vector2(this.#dir.x, -this.#dir.y)
            let rotatedDeltaP = new Vector2(rotationVec.dot(deltaP), rotationVec.cross(deltaP))
            this.#pos = this.#pos.add(rotatedDeltaP)

            // Update theta and direction vector
            this.#theta += alpha
            this.#dir.set(Math.cos(this.#theta), Math.sin(this.#theta))
        }
    }

    // Resolves collisions with environment
    resolveCollision(boid) {
        // Resolve collisions with other boids
        let dp = boid.#pos.sub(this.#pos)
        if (dp.dot(dp) > 4*Boid.#radius*Boid.#radius || boid === this) return

        let diff = dp.scale(2*Boid.#radius/dp.mag - 1)
        boid.#pos = boid.#pos.add(diff.scale(0.5))
        this.#pos = this.#pos.sub(diff.scale(0.5))
    }

    drawBoid(ctx) {
        // Apply transformation to object coordinates
        ctx.save()
        ctx.translate(this.#pos.x, this.#pos.y)
        ctx.rotate(this.#theta)
        ctx.strokeStyle = "#eee"
        
        // Draw boid
        ctx.stroke(Boid.#pathRobot)

        // Revert transformation to previous coordinates
        ctx.restore()
    }

    drawBoundingCircle(ctx) {
        // Apply transformation to object coordinates
        ctx.save()
        ctx.translate(this.#pos.x, this.#pos.y)
        ctx.rotate(this.#theta)
        ctx.strokeStyle = "#eee"
        
        // Draw bounding circle
        ctx.stroke(Boid.#pathBound)

        // Revert transformation to previous coordinates
        ctx.restore()
    }

    drawDetectionArea(ctx) {
        // Apply transformation to object coordinates
        ctx.save()
        ctx.translate(this.#pos.x, this.#pos.y)
        ctx.rotate(this.#theta)
        ctx.strokeStyle = "#eee"
        
        // Draw detection area
        ctx.stroke(Boid.#pathArea)

        // Revert transformation to previous coordinates
        ctx.restore()
    }
}


class Swarm {

    // Map for storing various demos
    static #demos = {}

    // Stores a demo in the map
    static registerDemo(demoName, demoClass) {
        if (typeof demoClass !== "function" || demoClass.length !== 1) throw new Error(`Invalid demoClass: Got ${demoClass}, expected constructor function with 1 parameter`)
        if (demoClass.prototype.draw !== undefined && typeof demoClass.prototype.draw !== "function") throw new Error(`Invalid demoClass draw type: Got ${demoClass.draw}, expected undefined or function`)
        if (typeof demoName !== "string") throw new Error(`Invalid demoName type: Got ${demoName}, expected string`)
        if (this.#demos[demoName] !== undefined) throw new Error(`Duplicate demoName error: ${demoName} is already registered`)
        this.#demos[demoName] = demoClass
    }
    
    // Private properties
    #lastUpdateTime = Date.now()
    #canvas
    #demo

    // Creates canvas on page and loads a named demo
    constructor(width, height, demoName) {
        this.#createDisplay(width, height)

        // Define scaling factor
        this.viewCenter = new Vector2
        this.viewScale = 10

        // Load demo
        this.boids = []
        this.#demo = {}
        if (typeof demoName === "string") this.loadDemo(demoName)
    }

    #createDisplay(width, height) {
        // Create container box
        let box = document.createElement("div")
        box.style.display = "flex"
        box.style.flexWrap = "wrap"
        
        // Create canvas
        this.#canvas = document.createElement("canvas")
        this.#canvas.style.flex = "0 1280px"
        this.#canvas.width = width
        this.#canvas.height = height
        this.#canvas.style.backgroundColor = "#222"
        box.appendChild(this.#canvas)

        // Create demo dropdown
        let dropdown = document.createElement("select")
        dropdown.onchange = event => this.loadDemo(dropdown.value)

        // Add options to dropdown
        for (const demoName in Swarm.#demos) {
            let option = document.createElement("option")
            option.value = demoName
            option.innerHTML = demoName
            dropdown.appendChild(option)
        }

        // Create control panel
        let panel = document.createElement("div")
        panel.style.display = "flex"
        panel.style.flexDirection = "column"
        panel.style.flex = "1 100px"
        panel.style.padding = "5px 0 0 5px"
        panel.appendChild(dropdown)
        box.appendChild(panel)

        // Add box to body
        document.body.appendChild(box)

        // Center coordinate system and flip y
        let ctx = this.#canvas.getContext("2d")
        ctx.translate(this.#canvas.width/2, this.#canvas.height/2)
        ctx.scale(1, -1)
    }

    // Loads demo from map by name
    loadDemo(demoName) {
        if (Swarm.#demos[demoName] === undefined) throw new Error(`Swarm demo ${demoName} is not registered`)
        
        this.boids = []
        this.#demo = new Swarm.#demos[demoName](this)
    }

    // Adds a boid to the swarm if not already added
    addBoid(x, y, theta, rule) {
        if (x instanceof Boid) {
            if (this.boids.indexOf(x) !== -1) return
            this.boids.push(x)
        } else this.boids.push(new Boid(x, y, theta, rule))
    }

    // Performs an iteration of the simulation and requests the next iteration
    run() {
        // Iteration
        this.#update()
        this.#draw()

        // Request next
        requestAnimationFrame(this.run.bind(this))
    }

    // Updates boid movement rules, positions, and resolves collisions
    #update() {
        // Calculate delta time
        let now = Date.now()
        let dt = Math.min(now - this.#lastUpdateTime, 100)/1000
        this.#lastUpdateTime = now

        // Update movement rules
        for (const boid of this.boids)
            boid.update(this.boids, dt)

        // Integrate based on delta time
        for (const boid of this.boids)
            boid.integrate(dt)

        // Resolve collisions between boids
        let len = this.boids.length
        for (let i = 0; i < 10; i++) 
            for (let first = 0; first < len; first++) 
                for (let second = first + 1; second < len; second++)
                    this.boids[first].resolveCollision(this.boids[second])
    }

    // Draws swarm
    #draw() {
        // Clear screen
        const ctx = this.#canvas.getContext("2d")
        ctx.clearRect(-this.#canvas.width/2, -this.#canvas.height/2, this.#canvas.width, this.#canvas.height)

        // Apply view parameters
        ctx.save()
        ctx.scale(this.viewScale, this.viewScale)
        ctx.translate(-this.viewCenter.x, -this.viewCenter.y)
        ctx.lineWidth = 1/this.viewScale
        
        // Draw swarm
        for (const boid of this.boids) {
            boid.drawBoid(ctx)
        }

        // Call demo draw function if it exists
        this.#demo.draw?.(ctx, this)

        // Revert scalaing
        ctx.restore()
    }
}

// Start a Swarm instance using "Example" setup when the window loads
window.onload = () => new Swarm(1280, 720, "Follow the Leader").run()
