import React, {useEffect, useReducer, useState} from "react";

function ActionInput({type, defaultValue, onChange}) {
    if (type === "float" || type === "int") {
        const onChangeInput = (e) => onChange(parseFloat(e.target.value))
        return <input
            onChange={onChangeInput}
            type="number"
            step="any"
            defaultValue={defaultValue}
            placeholder={type}
        />;
    }

    const onChangeInput = (e) => onChange(e.target.value)
    return <input
        onChange={onChangeInput}
        defaultValue={defaultValue}
        type="text"
        placeholder={type}
    />;
}

function ActionForm({args, lastArgs, onSubmit}) {
    const [formState, dispatch] = useReducer((state, action) => {
        return {...state, [action.key]: action.value}
    }, {})

    const onChange = (key) => (value) => dispatch({key: key, value: value})

    const onSubmitForm = async (event) => {
        event.preventDefault();
        onSubmit(formState)
    }

    useEffect(() => {
        for (let arg of Object.keys(args)) {
            dispatch({
                key: arg,
                value: lastArgs[arg],
            })
        }
    }, [args, lastArgs])
    return <form onSubmit={onSubmitForm}>
        <ul>
            {Object.keys(args).map(arg => {
                return (
                    <li key={arg}>
                        <label htmlFor={arg}>{arg}</label>:
                        <ActionInput
                            name={arg}
                            type={args[arg]}
                            onChange={onChange(arg)}
                            defaultValue={lastArgs[arg]}
                        />
                    </li>
                )
            })}
        </ul>
        <input type="submit"/>
    </form>

}

export default function ActionsView({actionURL}) {
    const [actions, setActions] = useState([])

    async function executeAction(name, args) {
        console.log("sending action", name, args)
        await fetch(actionURL, {
            method: "POST",
            body: JSON.stringify({
                name: name,
                args: args,
            })
        })
    }

    useEffect(() => {
        async function fetchActions(url) {
            const result = await fetch(url)
            const data = await result.json();
            console.log(data)
            setActions(data.functions)
        }

        fetchActions(actionURL)
    }, [actionURL])


    return <div>
        <ul>
            {actions.map((action) => {
                const onSubmit = async (data) => {
                    await executeAction(action.name, data)
                }
                return (<li key={action.name}>
                    <ul>
                        <li><strong>Name</strong>: {action.name}</li>
                        <li><strong>Documentation</strong>: {action.documentation}</li>
                        <li><strong>Arguments:</strong><ActionForm args={action.args} lastArgs={action.last_args_sent} onSubmit={onSubmit}/></li>
                    </ul>
                </li>);
            })}
        </ul>
    </div>
}