import {useEffect, useState} from "react";

// Every 1/REPLAY_RATE second, it will send all the new events (batched) to the client
// for displaying.
const REPLAY_RATE = 30

// NOTE: Live is limited by the backend.

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
        setEvent$(JSON.parse(msg.data));
    };
}


async function fetchReplay(setEvent$, url) {
    console.log("Fetching remote replay");
    const result = await fetch(url);
    const data = await result.json();
    const events = data.events;
    console.log("Fetched remote replay!")

    let i = 0;
    let j = 0;
    const startTime = new Date();
    const streamTimeOffset = events[0].time
    while (j < events.length) {
        const now = new Date();
        while (j < events.length && events[j].time - streamTimeOffset < (now - startTime) / 1000 + 1 / REPLAY_RATE) {
            j++
        }
        setEvent$(events.slice(i, j));
        i = j;
        await new Promise(resolve => setTimeout(resolve, 1000 / REPLAY_RATE));
    }
}

