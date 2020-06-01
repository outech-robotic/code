import React, {useMemo} from "react";
import {HashRouter, Link, Redirect, Route, Switch} from "react-router-dom";
import EncoderGraphView from "./EncoderGraphView";
import useEventStream from "../hooks/stream";
import ActionsView from "./ActionsView";
import "./App.css"
import SimulationView from "./SimulationView";
import useFilterEvent from "../hooks/useFilterEvent";
import {getLast, getValue, mapEventsToPoint} from "../event";
import useAggregatedGraph from "../hooks/useAggregatedGraph";
import useLast from "../hooks/useLast";

const useDataSeries = (event$, name) => {
    const events = useFilterEvent(event$, name);
    const xy = useMemo(() => mapEventsToPoint(events), [events])
    return useAggregatedGraph(xy);
}

function DebugInterface({event$, actionURL}) {
    const configurationEvents = useFilterEvent(event$, "configuration")
    const configuration = getValue(useLast(configurationEvents))

    const encoderLeft = useDataSeries(event$, "encoder_left")
    const speedLeft = useDataSeries(event$, "speed_left")
    const encoderRight = useDataSeries(event$, "encoder_right")
    const speedRight = useDataSeries(event$, "speed_right")

    const robotPositionEvent = useLast(useFilterEvent(event$, "position"))
    const robotAngleEvent = useLast(useFilterEvent(event$, "angle"))

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


