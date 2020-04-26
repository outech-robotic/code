import {useEffect, useMemo, useReducer} from "react";

function delta(lastValues) {
    if (lastValues.length <= 1) {
        return undefined
    }
    const value = lastValues[lastValues.length - 1];
    return {
        x: value.x,
        y: (value.y - lastValues[0].y) / (value.x - lastValues[0].x),
    }
}

export default function useDerivative(value) {
    const [lastValues, dispatch] = useReducer((state, action) => {
        if (action === undefined) {
            return state
        }
        return [...state, action].slice(-2);
    }, [])

    useEffect(() => {
        dispatch(value)
    }, [value])

    return useMemo(() => delta(lastValues), [lastValues])
}