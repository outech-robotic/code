import {useEffect, useState} from "react";

export default function useFilterEvent(event$, key) {
    const [filteredEvent$, setFilteredEvent$] = useState()
    useEffect(() => {
        if (event$ !== undefined) {
            setFilteredEvent$(event$.filter(e => e.key === key))
        }
    }, [event$, key])

    return filteredEvent$
}