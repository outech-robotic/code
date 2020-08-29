import React, {useState} from "react";
import './SimulationView.css'
import plateauImg from '../plateau.svg'
import robotImg from '../robot.svg'
import {getValue} from "../event";
import useInterval from "../hooks/usePeriodicInterval";
import ReactJson from 'react-json-view'


const WIDTH = 3000;
const HEIGHT = 2000;
const FPS = 60;

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

function drawGraph(ctx, nodes) {
    if (nodes === undefined) {
        console.log("nodes undefined")
        return
    }
    ctx.save();
    ctx.fillStyle = "rgba(240,0,200,0.4)";
    ctx.strokeStyle = "rgba(240,0,100,0.8)";
    nodes.forEach((node) => {
        ctx.beginPath();
        ctx.arc(node.position.x, HEIGHT - node.position.y, 10, 0, 2*Math.PI);
        if (node.neighbours !== undefined) {
            node.neighbours.filter(neighbourKey => neighbourKey > node.id)
                            .forEach((neighbourKey) => {
                                var neighbour = nodes[neighbourKey];
                                ctx.moveTo(node.position.x, HEIGHT - node.position.y);
                                ctx.lineTo(neighbour.position.x, HEIGHT - neighbour.position.y);
                                ctx.stroke();
                            });
        }
        ctx.fill();
    });
    ctx.restore();
}

export default function SimulationView({robotPositionEvent, robotAngleEvent, configuration, graph}) {
    const boardImageRef = React.useRef()
    const robotImageRef = React.useRef()

    const robotPosition = getValue(robotPositionEvent)
    const robotAngle = getValue(robotAngleEvent)
    const robotState = {
        position: robotPosition,
        angle: robotAngle
    }

    const [filterGraph, setFilterGraph] = useState(false);

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
        if (graph !== undefined && !filterGraph) {
            drawGraph(ctx, graph.nodes);
        }
        ctx.restore();
    }, 1000 / FPS);

    return <div>
        <section className="column-left">
            <section className="json-panel">
                <ReactJson src={robotState} displayDataTypes={false} displayObjectSize={false}/>
            </section>
            <section className="control-panel">
                <label>
                    <input type="checkbox" checked={filterGraph} onChange={(e) => setFilterGraph(e.target.checked)}/>
                    Filter graph
                </label>
            </section>
        </section>
        <section className="main-view">
            <canvas id="robotField" width="3000" height="2000">
            This page does not work, get a better (newer) browser.
            </canvas>
            <img alt="plateau" ref={boardImageRef} src={plateauImg} style={{display: "none"}}/>
            <img alt="robot" ref={robotImageRef} src={robotImg} style={{display: "none"}}/>
        </section>
    </div>
}
