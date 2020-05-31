import React from "react";
import './SimulationView.css'
import plateauImg from '../plateau.svg'
import robotImg from '../robot.svg'
import {getValue} from "../event";
import useInterval from "../hooks/usePeriodicInterval";


const WIDTH = 3000;
const HEIGHT = 2000;
const FPS = 60;
//
//
// let robot_width = 0;
// let robot_height = 0;
//
// let getFrame = null;
//
// async function init() {
//     const params = new URLSearchParams(window.location.search);
//     const replayURL = params.get('replay');
//     if (!replayURL) {
//         return
//     }
//     if (replayURL.startsWith('ws://')) {
//         let socket = new WebSocket(replayURL);
//         let position = {
//             "x": 0,
//             "y": 0,
//         }
//         let angle = 0;
//         socket.onmessage = function (event) {
//             let currentFrame = JSON.parse(event.data);
//             for (let f of currentFrame) {
//                 if (f.key === "position") {
//                     position = f.defaultValue;
//                 }
//                 if (f.key === "angle") {
//                     angle = f.defaultValue;
//                 }
//             }
//         };
//         robot_width = 240;
//         robot_height = 380;
//         getFrame = function () {
//             return {
//                 "position": position,
//                 "angle": angle,
//             }
//         }
//     } else if (replayURL.startsWith("http")) {
//         const simulation = await loadReplay(replayURL);
//
//         const config = simulation.events.find(e => e.key == 'configuration').defaultValue;
//         robot_width = config.robot_length;
//         robot_height = config.robot_width;
//
//         let frames = simulation.events;
//         let start_time = Date.now();
//         let replay_cursor = 0;
//         let position = {
//             "x": 0,
//             "y": 0,
//         }
//         let angle = 0;
//         getFrame = function () {
//             const t = Date.now() - start_time;
//             while (replay_cursor + 1 < frames.length && t > frames[replay_cursor + 1].time * 1000) {
//                 let f = frames[replay_cursor];
//                 if (f.key === "position") {
//                     position = f.defaultValue;
//                 }
//                 if (f.key === "angle") {
//                     angle = f.defaultValue;
//                 }
//                 replay_cursor += 1;
//             }
//             return {
//                 "position": position,
//                 "angle": angle,
//             }
//         };
//     } else {
//         return
//     }
//     window.requestAnimationFrame(draw);
// }

// window.addEventListener('load', init);

// function draw() {
//     const robot = getFrame();
//
//     const canvas = document.getElementById('robotField');
//     if (!canvas.getContext) {
//         return;
//     }
//     const ctx = canvas.getContext('2d');
//
//     ctx.globalCompositeOperation = 'source-over';
//     ctx.clearRect(0, 0, WIDTH, HEIGHT);
//     ctx.save();
//
//     drawBackground(ctx);
//
//     drawGrid(ctx, robot.obstacle_grid);
//
//     drawRobot(ctx, robot.position, robot.angle);
//
//     var list_obstacles = robot['position_obstacles'];
//
//     if (list_obstacles !== undefined) {
//         for (var obstacle of list_obstacles) {
//             drawObstacle(ctx, obstacle.x, obstacle.y);
//         }
//     }
//
//     ctx.restore();
//
//     window.requestAnimationFrame(draw);
// }
//
//
//
// function drawGrid(ctx, grid = []) {
//     ctx.fillStyle = "rgba(255, 0, 0, 0.5)";
//     for (var [x, row] of grid.entries()) {
//         for (var y = 0; y < row.length; y++) {
//             if (row[y] === '1') {
//                 ctx.fillRect(x * 10, HEIGHT - y * 10, 10, 10);
//             }
//         }
//     }
// }
//
function drawRobot(ctx, pos, angle, robot_width, robot_height, img) {
    if (pos === undefined) {
        return
    }
    const posX = pos.x;
    const posY = pos.y;
    ctx.save();
    ctx.translate(posX, HEIGHT - posY);
    ctx.rotate(-angle);

    ctx.beginPath();
    ctx.strokeStyle = "red";
    ctx.lineWidth = 4;
    ctx.strokeRect(-robot_width / 2, -robot_height / 2, robot_width, robot_height);
    ctx.stroke()

    ctx.rotate(-Math.PI / 2);
    ctx.drawImage(img, -robot_height / 2, -robot_width / 2, robot_height, robot_width)

    ctx.beginPath();
    ctx.strokeStyle = "white";
    ctx.lineWidth = 4;

    ctx.moveTo(0, -robot_width / 4);
    ctx.lineTo(0, robot_width / 4);

    ctx.lineTo(-20, robot_width / 4 - 40);

    ctx.moveTo(0, robot_width / 4);
    ctx.lineTo(20, robot_width / 4 - 40);

    ctx.stroke()

    ctx.restore();
}

//
//
// function drawObstacle(ctx, posX, posY) {
//     ctx.save();
//
//     ctx.beginPath();
//     ctx.arc(posX, HEIGHT - posY, 5, 0, 2 * Math.PI, true);
//     ctx.fillStyle = 'red';
//     ctx.fill();
//
//     ctx.strokeStyle = '#300000';
//     ctx.stroke();
//
//     ctx.restore();
// }
//


export default function SimulationView({robotPositionEvent, robotAngleEvent, configuration}) {
    const boardImageRef = React.useRef()
    const robotImageRef = React.useRef()

    const robotPosition = getValue(robotPositionEvent)
    const robotAngle = getValue(robotAngleEvent)

    useInterval(() => {
        const canvas = document.getElementById('robotField');
        if (!canvas.getContext) {
            return;
        }
        const ctx = canvas.getContext('2d');

        ctx.globalCompositeOperation = 'source-over';
        ctx.clearRect(0, 0, WIDTH, HEIGHT);
        ctx.save();

        ctx.drawImage(boardImageRef.current, 0, 0, WIDTH, HEIGHT);
        if (configuration !== undefined) {
            drawRobot(ctx, robotPosition, robotAngle, configuration.robot_length, configuration.robot_width, robotImageRef.current)
        }
        ctx.restore();
    }, 1000 / FPS);

    return <div>
        Robot position: {JSON.stringify(robotPositionEvent)}
        Robot angle: {robotAngle}
        <canvas id="robotField" width="3000" height="2000">
            This page does not work, get a better (newer) browser.
        </canvas>
        <img alt="plateau" ref={boardImageRef} src={plateauImg} style={{display: "none"}}/>
        <img alt="robot" ref={robotImageRef} src={robotImg} style={{display: "none"}}/>
    </div>

}
