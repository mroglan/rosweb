import { Box, Grid } from "@mui/material";
import { useState } from "react";
import Controls from './Controls'
import Display from "./Display";

export default function Main({ws}) {

    const [controls, setControls] = useState({
        topic: '',
        paused: false,
        selectedFilter: 'None',
        filters: {}
    })

    return (
        <Box>
            <Grid container spacing={1} justifyContent="center">
                <Grid item>
                    <Box>
                        <Controls ws={ws} controls={controls} setControls={setControls} />
                    </Box> 
                </Grid>
                <Grid item>
                    <Box>
                        <Display ws={ws} controls={controls} />
                    </Box>
                </Grid>
            </Grid>
        </Box>
    )
}