import { Box, Typography } from "@mui/material";
import { useEffect, useMemo, useRef, useState } from "react";

export default function Display({ws, controls}) {

    const [imgSize, setImgSize] = useState({width: 400, height: 400})

    const canvasRef = useRef();

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

        const ctx = canvasRef.current.getContext('2d')

        const rgba = new Uint8ClampedArray(info.data.width * info.data.height * 4)
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

        const imageData = new ImageData(rgba, info.data.width, info.data.height)

        ctx.putImageData(imageData, 0, 0)
    }, [ws.lastMessage])

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
        </Box>
    )
}