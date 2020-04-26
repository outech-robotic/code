import React from "react";

function download(filename, text) {
    var element = document.createElement('a');
    element.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
    element.setAttribute('download', filename);

    element.style.display = 'none';
    document.body.appendChild(element);

    element.click();

    document.body.removeChild(element);
}

export default function DownloadLink({title, data}) {
    const onClick = () => {
        const dataPerX = (() => {
            let dataPerX = {}
            for (let [i, series] of data.entries()) {
                for (let point of series) {
                    const previousValue = dataPerX[point.x] || {}
                    dataPerX[point.x] = {
                        ...previousValue,
                        [i]: point.y,
                    }
                }
            }
            return dataPerX
        })()

        const keys = Object.keys(dataPerX).map(key => parseFloat(key)).sort()

        const graph = keys.map(key => {
            const row = dataPerX[key]
            let result = [key]
            for (let i = 0; i < data.length; i++) {
                result.push(row[i])
            }
            return result
        })
        const resultCSV = [["time", ...title], ...graph].reduce((acc, value) => {
            return acc + value.join() + "\n"
        }, "")
        download('graph.csv', resultCSV)
    }
    return <button onClick={onClick}>Download CSV</button>
}
