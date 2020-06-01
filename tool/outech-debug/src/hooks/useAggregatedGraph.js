import {useEffect, useReducer} from "react";

export default function useAggregatedGraph(points) {
    const [graph, dispatch] = useReducer((state, action) => {
        if (action === undefined) {
            return state
        }
        return [...state, ...action]
    }, []);
    useEffect(() => {
        dispatch(points)
    }, [points])

    return graph
}

