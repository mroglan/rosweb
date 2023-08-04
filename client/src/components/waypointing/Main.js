import { Box, Grid } from "@mui/material";
import Controls from "./Controls";
import { useMemo, useState } from "react";
import Display from "./Display";

export default function Main({ws}) {

    const [controls, setControls] = useState({
        gpsTopic: '',
        odomTopic: ''
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