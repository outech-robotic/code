import React, {useRef, useState} from "react";

import CanvasJSReact from '../canvasjs.react';
import usePeriodicInterval from "../hooks/usePeriodicInterval";
// var CanvasJS = CanvasJSReact.CanvasJS;
var CanvasJSChart = CanvasJSReact.CanvasJSChart;

const MAX_POINT_COUNT_ON_GRAPH = 1000
const RENDER_FPS = 10


function getOptions(series, title, color) {
    return {
        zoomEnabled: true,
        animationEnabled: false,
        backgroundColor: "transparent",
        title: {
            text: title,
        },
        axisY: {
            includeZero: false,
            lineThickness: 1
        },
        data: [
            {
                type: "line",
                markerSize: 0,
                lineColor: color,
                lineDashType: "solid",
                dataPoints: series,
            },
        ],
    }
}


export default function Graph({value, seriesName, color}) {
    const chartRef = useRef(null)
    const [graphOptions, setGraphOptions] = useState(getOptions([], seriesName, color))

    usePeriodicInterval(() => {
        setGraphOptions((options) => {
            return {
                ...options,
                data: [{
                    ...options.data[0],
                    dataPoints: value.slice(-MAX_POINT_COUNT_ON_GRAPH),
                }]
            }
        })
        chartRef.current.render()
    }, 1000 / RENDER_FPS)


    return <div style={{height: "100%"}}>
        Number of points: {graphOptions.data[0].dataPoints.length}
        <CanvasJSChart options={graphOptions}
                       onRef={ref => chartRef.current = ref}
        />
    </div>
}
