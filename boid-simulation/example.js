//
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> MovementRule <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//  A MovementRule is a constructor (class) or object with the following methods:
//
//  - updateMovement(detectedBoids, dt)
//      This function describes the decision a Boid makes each frame given information
//      abouts its neighbors. The corresponding Boid object can be accessed using the
//      `this` keyword. The updateMovement function takes the following 2 parameters:
//
//      - detectedBoids
//          A list of boids in the detection area via the (hypothetical) vision system
//          Format: [
//              {
//                  Vector2 pos  - Vector containing position of detected boid (x=tangential, y=horizontal)
//                  Vector2 vel  - Vector containing relative velocity of boid (not implemented)
//                  number theta - Direction of detected boid relative 
//              }* (star meaning 0 or more in the list)
//          ]
//      - dt
//          A number containing the time elapsed (in seconds) since the last movement update
//
//
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> MovementRule EXAMPLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//

// Boid drives in lopsided circles
class Circles {
    updateMovement(detectedBoids, dt) {
        this.outputPower.set(-0.5 + Math.random(), 0.5 + Math.random())
    }
}

// Boid drives back and forth in a straight line
class Straight {

    time = 0

    updateMovement(detectedBoids, dt) {
        if (this.rule.time % 10 < 5)
            this.outputPower.set(1, 1)
        else
            this.outputPower.set(-1, -1)

        this.rule.time += dt
    }
}

// Boid turns away from detected boids
class Avert {
    updateMovement(detectedBoids, dt) {
        if (detectedBoids.length > 0) {
            this.outputPower.set(1, -1)
        } else this.outputPower.set(0, 0)
    }
}

//
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Boid class <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//  Represents a simulated boid. Boids need a MovementRule which describes the
//  decisions made each frame. The information given to the MovementRule is
//  designed to simulate the actual interface on the real system
//
//  Static Properties (accessed via Boid.)
//  - fov (read only, default PI/3)
//      Field of view of the detection area (aka camera fov)
//  - range (read only, default 30)
//      Maximum distance of the detection area
//  - size (read only)
//      Length of the boid's side
//  - wheelDiameter (read only)
//      Diameter of the wheel
//  - wheelBase (read only)
//      Distance between the left and right wheels
//  - encoderTicksPerRotation (read only)
//      Number of encoder ticks in one rotation
//  - encoderDistancePerTick (read only)
//      Wheel distance covered in one encoder tick
//
//  Static Methods (accessed via Boid.)
//  - setDetectionArea(fov, range)
//      Sets the detection area of the vision system to (fov, range)
//
//  Constructor
//  - new Boid(x, y, theta, rule)
//      Constructs a new boid at location (x, y) and angle theta
//      The MovementRule used to control the boid
//
//  Object Properties (accessed via instance of Boid)
//  - outputPower
//      Vector2 containing the power left and right (x, y) supplied to the motors
//  - encoder (read only)
//      Vector2 containing the encoder ticks for the left and right (x, y) motors
//  - movementRule
//      MovementRule used to update the boid's movement
//
//  Object Methods (accessed via instance of Boid)
//  - INTERNAL update(boids, dt)
//      Finds boids in detection area and passes result to movementUpdate
//  - INTERNAL integrate(dt)
//      Moves the boid one timestep using dt and velocity
//  - INTERNAL resolveCollision(boid)
//      Checks for and resolves a collision with another boid
//  - drawBoid(ctx)
//      Draws the boid on the given canvas context
//  - drawBoundingCircle(ctx)
//      Draws the boid's bounding circle on the given canvas context
//  - drawDetectionArea(ctx)
//      Draws the boid's detection area on the given canvas context
//
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Boid EXAMPLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//

// Boid construction
let exampleBoid = new Boid(0, -25, 0, Straight)
//                         ^x ^^^y ^t ^^^^^^^^MovementRule

// Forward full power
exampleBoid.outputPower.set(1, 1)

// Spin left full power
exampleBoid.outputPower.set(-1, 1)

// Stop if left encoder is larger than right
if (exampleBoid.encoder.x > exampleBoid.encoder.y) {
    exampleBoid.outputPower.set(0, 0)
}

// Change rule to Circles
exampleBoid.movementRule = Circles

//
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> SwarmDemo <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//  A SwarmDemo is a constructor (class) or object with the following methods:
//
//  - constructor(swarm)
//      The constructor function is called when initializing a demo. Place setup code
//      here. The Swarm instance loading the demo is passed as an argument
//  - draw(ctx, swarm) (optional)
//      Called after each draw with a CanvasRenderingContext2D and the swarm being drawn
//
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> MovementRule EXAMPLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//

class ExampleDemo {
    constructor(swarm) {
        // Add 5x3 rectangle of boids
        for (let x = 0; x < 5; x++)
        for (let y = 0; y < 3; y++)
            swarm.addBoid(10*x - 20, 10*y + 5, 0, Circles)
    
        // Add 3 more boids
        swarm.addBoid(-50, 20, 0, Straight)
        swarm.addBoid(0, -10, Math.PI/2, Avert)
        swarm.addBoid(exampleBoid)
    }

    // optional
    draw(ctx, swarm) {
        for (const boid of swarm.boids)
            if (boid.rule instanceof Avert)
                boid.drawDetectionArea(ctx)
    }
}

//
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Swarm class <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//  Static Methods (accessed via Swarm.)
//  - registerSetup(setupName, setupFunc<Swarm => void|updateFunc<Swarm,ctx => void>>)
//      Registers a setupFunc with the given setupName. Setup functions can be applied to
//          a Swarm object using loadSetup(setupName) which calls the setupFunc with the
//          swarm as an argument. This can be used to configure the swarm object. setupFunc
//          can also return an updateFunc which is called after each frame when the setup
//          is active
//      setupFunc is a function that take one parameter (Swarm) and returns nothing or an updateFunc
//      updateFunc is a function that is called after each draw call. It takes
//          two parameters (Swarm, CanvasRenderingContext2D) and returns nothing.
//          This is intended for updating the view parameters but can be used for much more
//  
//  YOU PROBABLY DO NOT NEED AN INSTANCE OF SWARM (everything below this line)
//
//  Constructor
//  - new Swarm(width, height, setupName)
//      Constructs a new Swarm with a given display width and height
//      The setupName is used to load a setup via loadSetup()
//
//  Public Properties (accessed vis Swarm instance)
//  - viewCenter
//      Vector2 containing the center point of the view
//  - viewScale
//      The scale factor of the view, increasing zooms in
//  - boids
//      Array containing the boids in the swarm indexed by id
//
//  Public Methods (accessed via Swarm instance)
//  - loadSetup(setupName)
//      Loads the setup registered under setupName and throws an error if not found
//  - addBoid(x, [y, theta, rule])
//      Adds a boid to the swarm. If x is a Boid object, it will be added
//      to the boids array. Otherwise, a new Boid object will be constructed
//      from the four parameters and added to the boids array
//  - run()
//      Starts the loop for the simulation. Only call once
//

// Register ExampleDemo under the name "Example"
Swarm.registerDemo("Example", ExampleDemo)

// Register ExampleDemo a second time under the name "Example2"
Swarm.registerDemo("Example2", ExampleDemo)

//
//  Files need to be listed inside of the index.html to be loaded
//  Refer to index.html to see the syntax for importing this file
//