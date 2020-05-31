import {useEffect, useState} from "react";

export default function useLast(events) {
    const [last, setLast] = useState(undefined)
    useEffect(() => {
        if (events === undefined || events.length === 0) {
            return
        }
        setLast(events[events.length - 1])
    }, [events])
    return last
}