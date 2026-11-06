# Newton's Laws Simulator

An interactive physics simulator for practicing Newton's laws. Students set up a system, predict the acceleration, tensions and contact forces, check their answers, and then watch the motion.

It is a static web page: plain HTML, CSS and JavaScript, with no build step and no dependencies.

## Systems

| Tab | What it models |
| --- | --- |
| **Atwood machine** | Two masses on one string over a massless, frictionless pulley. |
| **Modified Atwood** | 1–3 blocks on a table, tied by a string over a pulley at the edge to a hanging block. |
| **Inclined plane** | 1–3 blocks on a ramp at any angle (0–85°), with an optional hanging block over a pulley at the top. |
| **Blocks on a surface** | 1–4 blocks on a level surface, optionally with a string from the first block to the last. |

On every surface you can:

- set each block's mass;
- choose whether neighboring blocks are **touching** (contact forces, Newton's third-law pairs) **tied with a string** (tension), or **apart**;
- on a level surface with 3 or more blocks, tie a **long string from Block 1 to the last block** that passes over the blocks between (for example: Block 2 pushes Block 3, which pulls Block 1 along by the string);
- turn on **friction** with static (μs) and kinetic (μk) coefficients, either shared or set per block;
- add an **applied force** of any size and direction on any block (angled forces change the normal force);
- pick g for Earth, the Moon, Mars or Jupiter.

## For students

- **Predict & check**: type the acceleration, each tension, each contact force, and (optionally) normal, friction and net forces. Answers within 2% are marked correct; a wrong answer says whether it is too high or too low.
- **Free-body diagrams**: one diagram per object. Forces from the same string or contact share a color, and the action–reaction pairs are explained below the diagrams, including why W and N on the same block are *not* a third-law pair.
- **Motion graph**: position–, velocity– and acceleration–time graphs for any object, drawn as it moves.
- **Worked solution**: a step-by-step solution using the student's numbers. It stays locked until the student checks their answers or chooses *Show answers*.
- **Motion dots** leave a mark every 0.1 s so students can see the spacing grow, and the meter-stick scale lets them measure distances.

## For teachers

- **Copy link to this setup** puts the full setup in the URL, so you can assign an exact problem.
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
