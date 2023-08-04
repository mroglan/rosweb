import { Box, Grid, TextField, Typography } from "@mui/material";
import { useEffect, useMemo, useState } from "react";
import { BluePrimaryButton } from "../misc/buttons";
import { GROUP_COLORS } from "./Main";

export default function Add({ws, controls, setControls}) {

    const [currOdom, setCurrOdom] = useState(null)
    const [currNavSat, setCurrNavSat] = useState(null)
    const [color, setColor] = useState(0)

    useMemo(() => {
        if (!ws.lastMessage) return

        const msg = JSON.parse(ws.lastMessage.data)
        console.log(msg)

        if (msg.type !== 'stream') return

        if (msg.topics[controls.gpsTopic]) {
            setCurrNavSat(msg.topics[controls.gpsTopic])
        }
        if (msg.topics[controls.odomTopic]) {
            setCurrOdom(msg.topics[controls.odomTopic])
        }
    }, [ws.lastMessage])

    const handleGroupChange = (e) => {
        const val = Number(e.target.value)
        if (val < 1) return
        if (val > 20) return
        setControls({...controls, currGroup: val})
    }

    useMemo(() => {
        if (controls.groupColors[controls.currGroup]) {
            setColor(controls.groupColors[controls.currGroup])
            return 
        }
        const newColor = (controls.currGroup - 1) % GROUP_COLORS.length
        setColor(newColor)
    }, [controls.currGroup])

    useEffect(() => {
        setControls({...controls, groupColors: {...controls.groupColors, [controls.currGroup]: color}})
    }, [color])

    const handleChangeColor = () => {
        setColor((color + 1) % GROUP_COLORS.length)
    }

    return (
        <Box mt={3}>
            <Box width={200}>
                <BluePrimaryButton fullWidth>
                    Add Waypoint
                </BluePrimaryButton>
            </Box>
            <Box mt={3}>
                <Grid container alignItems="center">
                    <Grid item>
                        <Typography variant="h6">
                            To Group
                        </Typography>
                    </Grid>
                    <Grid item>
                        <Box ml={2}>
                            <TextField type="number" value={controls.currGroup} 
                                onChange={handleGroupChange} sx={{maxWidth: 75}} />
                        </Box>
                    </Grid>
                    <Grid item>
                        <Box ml={2}>
                            <Box width={50} height={55} bgcolor={GROUP_COLORS[color]} borderRadius={1}
                            sx={{cursor: 'pointer'}} onClick={handleChangeColor} />
                        </Box>
                    </Grid>
                </Grid>
            </Box>
        </Box>
    )
}