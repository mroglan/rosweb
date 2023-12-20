import React, { useMemo, useRef, useState } from 'react'
import { Box, Checkbox, FormControlLabel, Grid, TextField, Typography } from "@mui/material";
import { BluePrimaryButton } from "../misc/buttons";
import { ReadyState } from 'react-use-websocket'

export default function Main({ws}) {

    const [outputName, setOutputName] = useState('')
    const [bagPath, setBagPath] = useState('')
    const [topicName, setTopicName] = useState('')
    const [encoding, setEncoding] = useState('bgr8')
    const [createHTML, setCreateHTML] = useState(true)
    const [loading, setLoading] = useState(false)

    const [res, setRes] = useState({error: true, msg: ''})

    const handleOutputNameChange = (e) => {
        setOutputName(e.target.value)
    }

    const handleBagPathChange = (e) => {
        setBagPath(e.target.value)
    }

    const handleTopicNameChange = (e) => {
        setTopicName(e.target.value)
    }

    const handleEncodingChange = (e) => {
        setEncoding(e.target.value)
    }

    const handleCreateHTMLChange = (e) => {
        setCreateHTML(e.target.checked)
    }

    const handleConvert = (e) => {
        if (ws.readyState != ReadyState.OPEN) return

        setLoading(true)

        const req = {
            type: "request",
            operation: "bagged_image_to_video",
            data: {
                output_name: outputName || 'rosweb-output',
                bag_path: bagPath,
                topic_name: topicName,
                create_html: createHTML,
                encoding
            }
        }

        ws.sendMessage(JSON.stringify(req))
    }

    useMemo(() => {
        if (ws.readyState != ReadyState.OPEN) return

        if (!ws.lastMessage) return

        console.log(ws.lastMessage.data)

        const msg = JSON.parse(ws.lastMessage.data)

        if (msg.type !== 'response' || msg.operation !== 'bagged_image_to_video') return

        setRes({error: msg.data.status !== 200, msg: msg.data.msg})
        setLoading(false)
    }, [ws.lastMessage])

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
                <Box mt={3}>
                    <Grid container alignItems="center">
                        <Grid item>
                            <Typography variant="h6">
                                Encoding
                            </Typography>
                        </Grid>
                        <Grid item flex={1}>
                            <Box ml={4.9}>
                                <TextField value={encoding} onChange={handleEncodingChange}
                                    disabled={loading} sx={{width: 100}} />
                            </Box>
                        </Grid>
                    </Grid>
                </Box>
                <Box mt={3}>
                    <FormControlLabel label="Create HTML file to view video" 
                        control={<Checkbox checked={createHTML} onChange={handleCreateHTMLChange} />} />
                </Box>
                {res.msg && <Box mt={6}>
                    <Typography variant="h6" color={res.error ? "error.main" : "success.main"}>
                        {res.msg}
                    </Typography>
                </Box>}
                {res.error && <Box mt={6}>
                    <BluePrimaryButton sx={{minWidth: 200}} onClick={handleConvert}
                        disabled={loading}>
                        Convert
                    </BluePrimaryButton>
                </Box>}
            </Box>
        </Box>
    )     
}