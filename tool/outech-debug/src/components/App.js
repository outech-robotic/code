import React, {useMemo} from "react";
import {HashRouter, Link, Redirect, Route, Switch} from "react-router-dom";
import EncoderGraphView from "./EncoderGraphView";
import useEventStream from "../hooks/stream";
import ActionsView from "./ActionsView";
import "./App.css"
import SimulationView from "./SimulationView";
import useFilterEvent from "../hooks/useFilterEvent";
import {getValue, mapEventToPoint} from "../event";
import useDerivative from "../hooks/useDerivative";
import useAggregatedGraph from "../hooks/useAggregatedGraph";
import useDecimate from "../hooks/useDecimate";

const DATA_DECIMATION_RATE = 10

const useDataSeries = (event$, name) => {
    const event = useFilterEvent(event$, name);
    const decimatedEvent = useDecimate(event, DATA_DECIMATION_RATE)
    const xy = useMemo(() => mapEventToPoint(decimatedEvent), [decimatedEvent])
    const speedXY = useDerivative(xy);
    return [useAggregatedGraph(xy), useAggregatedGraph(speedXY)]
}

function DebugInterface({event$, actionURL}) {
    const configurationEvent = useFilterEvent(event$, "configuration")
    const configuration = getValue(configurationEvent)

    const [encoderLeft, speedLeft] = useDataSeries(event$, "encoder_left")
    const [encoderRight, speedRight] = useDataSeries(event$, "encoder_right")

    const robotPositionEvent = useFilterEvent(event$, "position")
    const robotAngleEvent = useFilterEvent(event$, "angle")

    return (
        <HashRouter>
            <div>
                <nav>
                    <ul>
                        <li>
                            <Link to="/actions">Actions</Link>
                        </li>
                        <li>
                            <Link to="/encoder_graph">Encoder graph</Link>
                        </li>
                        <li>
                            <Link to="/simulation">Simulation</Link>
                        </li>
                    </ul>
                </nav>

                <Switch>
                    <Route path="/simulation">
                        <SimulationView
                            robotPositionEvent={robotPositionEvent}
                            robotAngleEvent={robotAngleEvent}
                            configuration={configuration}
                        />
                    </Route>
                    <Route path="/actions">
                        <ActionsView actionURL={actionURL}/>
                    </Route>
                    <Route path="/encoder_graph">
                        <EncoderGraphView
                            encoderLeft={encoderLeft}
                            encoderRight={encoderRight}
                            speedLeft={speedLeft}
                            speedRight={speedRight}
                        />
                    </Route>
                    <Route path="/">
                        <Redirect to="/simulation"/>
                    </Route>
                </Switch>
            </div>
        </HashRouter>
    );
}


export default function App() {
    const urlParams = new URLSearchParams(window.location.search);
    const replayURL = urlParams.get('replay');
    const liveURL = urlParams.get('live');
    const actionURL = urlParams.get('action');

    const [url, live] = (() => {
        if (replayURL) {
            return [replayURL, false]
        } else if (liveURL) {
            return [liveURL, true]
        } else {
            return [undefined, undefined]
        }
    })()

    const event$ = useEventStream(url, live)

    if (!liveURL && !replayURL) {
        return <div>Please set either "live" or "replay" in query.</div>
    }

    if (liveURL && replayURL) {
        return <div>Cannot set both "replay" and "live" in query.</div>
    }

    return (
        <DebugInterface event$={event$} actionURL={actionURL}/>
    )
}


