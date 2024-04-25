
// Boid that acts as wall
class Wall {
    updateMovement(detectedBoids, dt) {
        // Does absolute nothing
    }
}

// Boid turns away from detected boids
class Follow {
    updateMovement(detectedBoids, dt) {
        if (detectedBoids.length > 0) {
            this.outputPower.set(0.1, -0.1)
        } else this.outputPower.set(0, 0)
    }
}

class FollowDemo {
    constructor(swarm) {
        // Add top and bottom walls
        for (let x = -6; x < 6; x++) {
            swarm.addBoid(10*x + 5, 35, -Math.PI/2, Wall)
            swarm.addBoid(10*x + 5, -35, Math.PI/2, Wall)
        }

        // Add left and right walls
        for (let y = -3; y <= 3; y++) {
            swarm.addBoid(-63, 10*y, 0, Wall)
            swarm.addBoid(63, 10*y, -Math.PI, Wall)
        }
    
        // Add boid
        swarm.addBoid(0, -10, Math.PI/2, Follow)
    }

    // optional
    draw(ctx, swarm) {
        for (const boid of swarm.boids)
            if (boid.rule instanceof Follow)
                boid.drawDetectionArea(ctx)
    }
}

Swarm.registerDemo("Follow the Leader", FollowDemo)