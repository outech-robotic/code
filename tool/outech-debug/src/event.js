export function getValue(event) {
    if (event === undefined) return undefined
    return event.value
}

export function mapEventsToPoint(events) {
    if (events === undefined) return
    return events.map(mapEventToPoint)
}

export function mapEventToPoint(event) {
    if (event === undefined) return
    return {
        x: event.time,
        y: event.value,
    }
}
