import React, { useMemo, useRef, useState } from 'react'
import { Box, Grid, TextField, Typography } from "@mui/material";
import { BluePrimaryButton } from "../misc/buttons";
import { ReadyState } from 'react-use-websocket'

export default function Main({ws}) {

    const [outputName, setOutputName] = useState('')
    const [bagPath, setBagPath] = useState('')
    const [topicName, setTopicName] = useState('')
    const [loading, setLoading] = useState(false)

    const handleOutputNameChange = (e) => {
        setOutputName(e.target.value)
    }

    const handleBagPathChange = (e) => {
        setBagPath(e.target.value)
    }

    const handleTopicNameChange = (e) => {
        setTopicName(e.target.value)
    }

    const handleConvert = (e) => {
        if (ws.readyState != ReadyState.OPEN) return;

        setLoading(true)

        const req = {
            type: "request",
            operation: "bagged_image_to_video",
            data: {
                outputName: outputName || 'rosweb-output',
                bagPath,
                topicName
            }
        }

        ws.sendMessage(JSON.stringify(req))
    }

    return (
        <Box mt={3}>
            <Box>
                <Box>
                    <Grid container alignItems="center">
                        <Grid item>
                            <Typography variant="h6">
                                Bag Path
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
                                <TextField disabled={loading} value={bagPath}
                                    onChange={handleBagPathChange} fullWidth />
                            </Box>
                        </Grid>
                    </Grid>
                </Box>
                <Box mt={3}>
                    <Grid container alignItems="center">
                        <Grid item>
                            <Typography variant="h6">
                                Output Path
                            </Typography>
                        </Grid>
                        <Grid item>
                            <Box ml={2}>
                                <Typography variant="body1">
                                    ~/Downloads/
                                </Typography>
                            </Box>
                        </Grid>
                        <Grid item>
                            <Box ml={1}>
                                <TextField placeholder="rosweb-output" value={outputName}
                                onChange={handleOutputNameChange} disabled={loading} />
                            </Box>
                        </Grid>
                        <Grid item>
                            <Box ml={1}>
                                <Typography variant="body1">
                                    .mp4
                                </Typography>
                            </Box>
                        </Grid>
                    </Grid>
                </Box>
                <Box mt={3}>
                    <Grid container alignItems="center">
                        <Grid item>
                            <Typography variant="h6">
                                Topic Name
                            </Typography>
                        </Grid>
                        <Grid item flex={1}>
                            <Box ml={2}>
                                <TextField value={topicName} onChange={handleTopicNameChange}
                                    disabled={loading} fullWidth />
                            </Box>
                        </Grid>
                    </Grid>
                </Box>
                <Box mt={6}>
                    <BluePrimaryButton sx={{minWidth: 200}} onClick={handleConvert}
                        disabled={loading}>
                        Convert
                    </BluePrimaryButton>
                </Box>
            </Box>
        </Box>
    )     
}