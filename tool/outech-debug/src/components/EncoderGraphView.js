import React from "react";
import Graph from "./Graph";
import './EncoderGraphView.css';
import DownloadLink from "./DownloadLink";


export default function EncoderGraphView({encoderLeft, encoderRight, speedLeft, speedRight}) {
    return (
        <div id="graphContainer">
            <DownloadLink title={["encoder_left", "encoder_right"]} data={[encoderLeft, encoderRight]}/>
            <div className="graphRow">
                <div className="graphElement">
                    <Graph value={encoderLeft} seriesName='Position left' color='royalblue'/>
                </div>
                <div className="graphElement">
                    <Graph value={encoderRight} seriesName='Position right' color='salmon'/>
                </div>
            </div>
            <div className="graphRow">
                <div className="graphElement">
                    <Graph value={speedLeft} seriesName='Speed left' color='royalblue'/>
                </div>
                <div className="graphElement">
                    <Graph value={speedRight} seriesName='Speed right' color='salmon'/>
                </div>
            </div>
        </div>
    )
}
