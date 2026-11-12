# Newton's Laws Simulator

An interactive physics simulator for practicing Newton's laws. Students set up a system, predict the acceleration, tensions and contact forces, check their answers, and then watch the motion.

It is a static web page: plain HTML, CSS and JavaScript, with no build step and no dependencies.

## Systems

| Tab | What it models |
| --- | --- |
| **Atwood machine** | Two masses on one string over a massless, frictionless pulley. |
| **Modified Atwood** | 1–3 blocks on a table, tied by a string over a pulley at the edge to a hanging block. |
| **Inclined plane** | 1–3 blocks on a ramp at any angle (0–85°). The low end faces right. Optional hanging block over a pulley at the top, or a launch off the table at the bottom as a projectile. |
| **Blocks on a surface** | 1–4 blocks on a level surface, optionally with a string from the first block to the last. |
| **Elevator** | A person on a scale in an elevator: speeding up, slowing down or at constant speed (up or down), a full ride, or free fall. Optional cable tension with the car's mass. |
| **Accelerating car** | An ornament hanging from the rearview mirror tilts to tan θ = a/g while the car speeds up or brakes. Optional hidden acceleration: read the angle off a protractor and work out a. |
| **Human body** | OpenStax-style problems: a leg in traction (pulleys at two angles), jumping, landing from a height, and a sprint start. |
| **Equilibrium** | A traffic light hanging from two cables at different angles, a block pressed against a wall (static or sliding, with a push at any angle), and a block and tackle with 1–6 rope segments (hold, lift steadily, or lift while speeding up). |
| **Labs** | *F = ma:* a cart and hanging mass with photogates; each run is recorded and the F-vs-a graph's slope gives the total mass and its intercept the friction. *Tilt the ramp:* the ramp rises until the block slips (μs = tan θ), then a timed slide gives μk. Both offer hidden values with measurement noise. |
| **Third-law puzzles** | Horse and cart, tug-of-war, truck vs car collision, and book on a table: multiple-choice concept questions plus numbers, with third-law pairs shown. |
| **Chaos** | A single pendulum and a double pendulum, computed with 4th-order Runge–Kutta, each with a live phase portrait (θ vs ω). An optional twin started 0.1° away shows sensitivity to initial conditions. Systems are self-contained in `CHAOS_SYSTEMS` in `js/app.js`, so more can be added. |

On every surface you can:

- set each block's mass;
- choose whether neighboring blocks are **touching** (contact forces, Newton's third-law pairs), **tied with a string** (tension), or **apart**;
- on a level surface with 3 or more blocks, tie a **long string from Block 1 to the last block** that passes over the blocks between (for example: Block 2 pushes Block 3, which pulls Block 1 along by the string);
- **stack a block on top** of any block, with its own μs and μk between the blocks; the simulator tests whether it rides along or slips;
- turn on **friction** with static (μs) and kinetic (μk) coefficients, either shared or set per block;
- add an **applied force** of any size and direction on any block, including a stacked block (angled forces change the normal force);
- pick g for Earth, the Moon, Mars or Jupiter.

## For students

- **Predict & check**: type the acceleration, each tension, each contact force, and (optionally) normal, friction and net forces. Answers within 2% are marked correct; a wrong answer says whether it is too high or too low.
- **Free-body diagrams**: one diagram per object. Forces from the same string or contact share a color, and the action–reaction pairs are explained below the diagrams, including why W and N on the same block are *not* a third-law pair.
- **Motion graph**: position–, velocity– and acceleration–time graphs for any object, drawn as it moves.
- **Worked solution**: a step-by-step solution using the student's numbers. It stays locked until the student checks their answers or chooses *Show answers*.
- **Launch off the table** (inclined plane): the block slides a distance d down the ramp, then a smooth lip turns it horizontal (or a small kicker ramp turns it to a launch angle φ you choose), and it flies off a table of height H as a projectile. The kicker's size is ignored in the solution. Students predict the launch speed, time in the air and landing distance; the graph shows x, vₓ and v_y against time.
- **Mystery mass**: one mass is hidden. Students measure the motion with the clock, ruler, readouts and graphs, then work out the mass (5% tolerance). Force values, the acceleration graph and the worked solution stay hidden until *Show answers*.
- **Motion dots** leave a mark every 0.1 s so students can see the spacing grow, and the meter-stick scale lets them measure distances.

## For teachers

- **Copy link to this setup** puts the full setup in the URL, so you can assign an exact problem. With **Mystery mass** on, the hidden value is left out of the link, so each student who opens it gets a different mass.
- **New random problem** gives fresh numbers with the same arrangement of blocks.

## Physics notes

- Strings and pulleys are massless and frictionless. Every run starts from rest.
- The engine tests static friction before anything moves, then uses kinetic friction once the system slides.
- It handles strings going slack and blocks separating. For example, pulling the back block away from a touching pair leaves the front block behind, and pushing a string-linked chain from behind leaves the string slack.
- When a chain is held at rest by static friction on several blocks, internal tensions are not uniquely determined. The simulator says so instead of grading those answers.
- Sign conventions: + is to the right or up the incline for blocks on a surface, down for a hanging block, and up for mass 1 / down for mass 2 on the Atwood machine.

## Running it

Open `index.html` in a browser, or publish with GitHub Pages: **Settings → Pages → Deploy from a branch → `main` / root**.

Physics tests (Node 18+):

```bash
node --test
```

## Files

- `index.html`: page structure
- `css/style.css`: styles, including light and dark themes
- `js/physics.js`: physics engine (no DOM, unit-tested)
- `js/app.js`: controls, animation, diagrams, graphs, checking and worked solutions
- `tests/physics.test.js`: tests against textbook results

## Updating the site

`index.html` loads the CSS and JavaScript with a version tag (for example `css/style.css?v=202611091530`). When you change those files, update the tag so browsers download the new copies instead of using old cached ones.
