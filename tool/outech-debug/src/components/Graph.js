import React from "react";
import {Scatter} from "react-chartjs-2";
// noinspection ES6UnusedImports
import 'chartjs-plugin-zoom'

const options = {
    pan: {
        enabled: true,
        mode: 'xy'
    },
    zoom: {
        enabled: true,
        mode: 'xy'
    },
    maintainAspectRatio: false,
    animation: {
        duration: 0 // general animation time
    },
    hover: {
        animationDuration: 0 // duration of animations when hovering an item
    },
    responsiveAnimationDuration: 0, // animation duration after a resize
    elements: {
        line: {
            tension: 0 // disables bezier curves
        }
    }
}


export default function Graph({value, seriesName, color}) {
    const graphRef = React.useRef();
    const data = {
        datasets: [
            {
                label: seriesName,
                fill: false,
                pointStyle: 'cross',
                borderColor: color,
                pointBorderColor: color,
                pointHoverRadius: 20,
                pointHoverBorderWidth: 2,
                pointRadius: 2,
                pointHitRadius: 10,
                showLine: true,
                borderWidth: 1,
                data: value.slice(-500),
            }
        ]
    };

    const onClick = () => graphRef.current.chartInstance.resetZoom()

    return <div style={{height: "100%"}}>
        <button type="button" onClick={onClick}>reset zoom</button>
        Number of points: {data.datasets[0].data.length}
        <Scatter ref={graphRef} data={data} options={options} color='blue'/>
    </div>
}
