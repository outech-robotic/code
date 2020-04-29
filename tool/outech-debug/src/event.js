export function getValue(event) {
    if (event === undefined) return undefined
    return event.value
}

export function mapEventToPoint(event) {
    if (event === undefined) return
    return {
        x: event.time,
        y: event.value,
    }
}
