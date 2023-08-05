import { Box, Grid, TextField, Typography } from "@mui/material";
import { useEffect, useState } from "react";
import { BluePrimaryButton } from "../misc/buttons";
import { ReadyState } from 'react-use-websocket'

export default function Export({ws, controls, setControls}) {

    const [dir, setDir] = useState(controls.saveDir)
    const [loading, setLoading] = useState(false)

    useEffect(() => {
        if (dir === controls.saveDir) return
        setControls({...controls, saveDir: dir})
    }, [dir])

    const exportWaypoints = () => {
        if (ws.readyState != ReadyState.OPEN) return

        setLoading(true)

        const req = {
            type: 'request',
            operation: 'save_waypoints',
            data: {
                waypoints: Object.keys(controls.waypoints).map(key => (
                    controls.waypoints[key].map(point => ({
                        pos: point.navsat,
                        orientation: point.orientation,
                        group: key
                    }))
                )).flat(1),
                groups: Object.keys(controls.waypoints),
                save_dir: dir
            }
        }
        console.log('req', req)
        ws.sendMessage(JSON.stringify(req))
    }

    return (
        <Box mt={3}>
            <Grid container alignItems="center">
                <Grid item>
                    <Typography variant="h6">
                        Directory
                    </Typography>
                </Grid>
                <Grid item>
                    <Box ml={2}>
                        <Typography variant="body1">
                            ~/
                        </Typography>
                    </Box>
                </Grid>
                <Grid item flex={1}>
                    <Box ml={1}>
                        <TextField value={dir} onChange={(e) => setDir(e.target.value)}
                            fullWidth />
                    </Box>
                </Grid>
            </Grid>
            <Box mt={6}>
                <Box width={200}>
                    <BluePrimaryButton fullWidth disabled={loading}
                        onClick={() => exportWaypoints()}>
                        Export
                    </BluePrimaryButton>
                </Box>
            </Box>
        </Box>
    )
}