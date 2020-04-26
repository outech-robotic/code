import {useEffect, useState} from "react";

export default function useFilterEvent(event$, key) {
    const [filteredEvent$, setFilteredEvent$] = useState()
    useEffect(() => {
        if (event$ !== undefined && event$.key === key) {
            setFilteredEvent$(event$)
        }
    }, [event$, key])

    return filteredEvent$
}