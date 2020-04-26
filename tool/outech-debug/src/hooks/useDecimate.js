import {useEffect, useState} from "react";

export default function useDecimate(event, rate) {
    const [currentValue, setCurrentValue] = useState()
    useEffect(() => {
        if (event === undefined) {
            return
        }
        if (currentValue === undefined) {
            setCurrentValue(event)
            return
        }
        if (event.time - currentValue.time > 1 / rate) {
            setCurrentValue(event)
        }
    }, [event, rate, currentValue])
    return currentValue
}