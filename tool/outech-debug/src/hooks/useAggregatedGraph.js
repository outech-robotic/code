import {useEffect, useReducer} from "react";

export default function useAggregatedGraph(point) {
    const [graph, dispatch] = useReducer((state, action) => {
        if (action === undefined) {
            return state
        }
        return [...state, action]
    }, []);
    useEffect(() => {
        dispatch(point)
    }, [point])

    return graph
}

