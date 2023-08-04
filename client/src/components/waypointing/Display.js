import { Box } from "@mui/material";
import { useRef, useState, useMemo } from "react";

export default function Display({ws}) {

    const [canvasSize, setCanvasSize] = useState({width: 800, height: 800})

    const canvasRef = useRef()

    useMemo(() => {
        if (!ws.lastMessage) return

        const msg = JSON.parse(ws.lastMessage.data)
        // console.log(msg)
    }, [ws.lastMessage])

    return (
        <Box>
            <Box width={canvasSize.width} height={canvasSize.height} border="1px solid #000">
                <canvas ref={canvasRef} width={canvasSize.width} height={canvasSize.height} />
            </Box>
        </Box>
    )
}