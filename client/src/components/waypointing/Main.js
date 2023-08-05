import { Box, Grid } from "@mui/material";
import Controls from "./Controls";
import { useMemo, useState } from "react";
import Display from "./Display";

export const GROUP_COLORS = [
    '#FF0000', // red,
    '#008000', // green
    '#0000FF', // blue 
    '#FF00FF', // pink
    '#800080', // purple
    '#00FF00', // lime
    '#808000', // olive
    '#FFFF00', // yellow
    '#800000' // maroon
]

export default function Main({ws}) {

    const [controls, setControls] = useState({
        gpsTopic: '',
        odomTopic: '',
        waypoints: {},
        groupColors: {},
        currGroup: 1,
        saveDir: ''
    })

    return (
        <Box>
            <Grid container spacing={1} wrap="nowrap" minWidth={1300}>
                <Grid item width={500}>
                    <Controls ws={ws} controls={controls} setControls={setControls} />
                </Grid>
                <Grid item minWidth={800} flex={1}>
                    <Display ws={ws} />
                </Grid>
            </Grid>
        </Box>
    )
}