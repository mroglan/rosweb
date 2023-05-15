import { Box, ImageList, Typography } from "@mui/material";
import { useEffect, useMemo, useRef, useState } from "react";
import { BluePrimaryButton } from "../misc/buttons";
import { convertRGB, withinBounds } from "./colorConversions";

export default function Display({ws, controls}) {

    const [imgSize, setImgSize] = useState({width: 400, height: 400})
    const [applyFilter, setApplyFilter] = useState(false)

    const canvasRef = useRef();
    const lastData = useRef()

    const updateCanvasImage = () => {
        const ctx = canvasRef.current.getContext('2d')

        if (!applyFilter) {
            const imageData = new ImageData(lastData.current, imgSize.width, imgSize.height)
            ctx.putImageData(imageData, 0, 0)
            return
        }

        const filter = controls.filters[controls.selectedFilter]
        const data = lastData.current
        const newData = new Uint8ClampedArray(imgSize.width * imgSize.height * 4)
        for (let i = 0; i < data.length; i += 4) {
            if (withinBounds(filter.lower, filter.upper, 
                convertRGB([data[i], data[i+1], data[i+2]], filter.type))) {
                newData[i] = data[i]
                newData[i+1] = data[i+1]
                newData[i+2] = data[i+2]
                newData[i+3] = data[i+3]
            } else {
                newData[i+3] = 0
            }
        }
        const imageData = new ImageData(newData, imgSize.width, imgSize.height)
        ctx.putImageData(imageData, 0, 0)
    }

    useMemo(() => {
        if (!ws.lastMessage) return
        if (!canvasRef.current) return

        const msg = JSON.parse(ws.lastMessage.data)

        if (msg.type !== 'stream') return

        const info = msg.topics[controls.topic]

        if (!info) return

        if (info.data.width !== imgSize.width) {
            setImgSize({width: info.data.width, height: info.data.height})
            return
        }

        const rgba = new Uint8ClampedArray(imgSize.width * imgSize.height * 4)
        let i = 0
        let j = 0
        while (i < info.data.data.length) {
            rgba[j] = info.data.data[i];
            if ((i + 1) % 3 === 0) {
                rgba[++j] = 255
            }
            i++
            j++
        }

        lastData.current = rgba
        updateCanvasImage()
    }, [ws.lastMessage])

    useMemo(() => {
        if (!lastData.current) return
        
        if (controls.paused) {
            updateCanvasImage()
        }
    }, [applyFilter, controls.filters])

    useMemo(() => {
        if (applyFilter && controls.selectedFilter === 'None') {
            setApplyFilter(false)
        }
    }, [controls])

    return (
        <Box width={imgSize.width}>
            <Box textAlign="center">
                <Typography variant="h6">
                    {controls.paused ? "Paused" : "Playing"}
                </Typography>
            </Box>
            <Box border="1px solid #000" height={imgSize.height}>
                <canvas ref={canvasRef} width={imgSize.width} height={imgSize.height} />
            </Box>
            <Box mt={3}>
                <BluePrimaryButton sx={{minWidth: 200}} 
                    onClick={() => controls.selectedFilter !== 'None' && setApplyFilter(!applyFilter)}>
                    {applyFilter ? 'Unapply Filter' : 'Apply Filter'}
                </BluePrimaryButton> 
            </Box>
        </Box>
    )
}