import { Box, Grid, TextField, Typography } from "@mui/material";
import React, { useState } from "react";
import WaypointCard from "./WaypointCard";

export default function Edit({controls, setControls}) {

    const [group, setGroup] = useState('')
    const [expanded, setExpanded] = useState([0,0])

    const handleGroupChange = (e) => {
        const val = Number(e.target.value)
        if (!val) {
            setGroup('')
            return
        }
        if (val < 0) return
        if (controls.groupColors[val] === undefined) return
        setGroup(val)
    }

    const unExpand = () => {
        setExpanded([0,0])
    }

    const expand = (group, num) => {
        setExpanded([group, num])
    }

    return (
        <Box height="calc(100vh - 280px)" sx={{overflowY: 'scroll'}}>
            <Box mt={3}>
                <Grid container alignItems="center">
                    <Grid item>
                        <Typography variant="h6">
                            Showing Group
                        </Typography>
                    </Grid>
                    <Grid item>
                        <Box width={100} ml={2}>
                            <TextField type="number" placeholder="All" fullWidth
                                value={group} onChange={handleGroupChange} />
                        </Box>
                    </Grid>
                </Grid>
            </Box>
            <Box mt={3}>
                {Object.keys(controls.waypoints).map(oGroup => (
                    !group || group === oGroup ? <Box key={oGroup}>
                        {controls.waypoints[oGroup].map((_, i) => (
                            <Box mt={3} key={i}>
                                <WaypointCard controls={controls} setControls={setControls}
                                    group={oGroup} num={i} 
                                    expanded={expanded[0] === oGroup && expanded[1] === i}
                                    unExpand={unExpand} expand={expand}
                                />
                            </Box>
                        ))}
                    </Box> : null
                ))}
            </Box>
        </Box>
    )
}