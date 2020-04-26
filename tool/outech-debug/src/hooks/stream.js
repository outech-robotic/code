import {useEffect, useState} from "react";

export default function useEventStream(url, live) {
    const [event$, setEvent$] = useState()
    useEffect(() => {
        if (live === undefined) return

        if (live) {
            fetchLive(setEvent$, url)
        } else {
            fetchReplay(setEvent$, url)
        }
    }, [url, live])

    return event$
}

function fetchLive(setEvent$, url) {
    console.log("Opening web socket");
    const socket = new WebSocket(url);

    socket.onopen = function () {
        console.log("Opened");
    }

    socket.onmessage = function (msg) {
        const eventList = JSON.parse(msg.data);
        for (let event of eventList) {
            setEvent$(event)
        }
    };
}


async function fetchReplay(setEvent$, url) {
    console.log("Fetching remote replay");
    const result = await fetch(url);
    const data = await result.json();
    const events = data.events;
    console.log("Fetched remote replay!")

    const streamTimeOffset = events[0].time
    const startTime = new Date();
    for (const event of data.events) {
        const event_time = event.time - streamTimeOffset;
        const cur_time = (new Date() - startTime) / 1000;
        const time_to_wait = event_time - cur_time;
        await new Promise(resolve => setTimeout(resolve, time_to_wait * 1000));
        setEvent$(event)
    }
}

