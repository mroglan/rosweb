import { Box, Grid, TextField, Typography, useMediaQuery } from "@mui/material";
import { useMemo, useState } from "react";
import SyncIcon from '@mui/icons-material/Sync';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import { BluePrimaryButton, BluePrimaryIconButton } from "../misc/buttons";

export default function Controls({ws, controls, setControls}) {

    const [typedTopic, setTypedTopic] = useState('')

    const [syncDisabled, setSyncDisabled] = useState(true)
    const [pauseDisabled, setPauseDisabled] = useState(true)

    useMemo(() => {
        setSyncDisabled(false)
        setPauseDisabled(false)
    }, [controls])

    const handleTypedTopicChange = (e) => {
        setTypedTopic(e.target.value)
    }

    const handleTopicChange = (e) => {
        if (controls.topic == typedTopic) return

        let msg = {
            type: 'request',
            data: {
                msg_type: 'sensor_msgs/msg/Image'
            }
        }
        if (controls.topic) {
            msg.operation = 'change_subscriber'
            msg.data.prev_topic_name = controls.topic
            msg.data.new_topic_name = typedTopic
        } else {
            msg.operation = 'create_subscriber'
            msg.data.topic_name = typedTopic
        }
        ws.sendMessage(JSON.stringify(msg))
        setSyncDisabled(true)
        setControls({...controls, topic: typedTopic}) 
    }

    const handlePauseChange = (e) => {
        let msg = {
            type: 'request',
            operation: 'toggle_pause_subscriber',
            data: {
                topic_name: controls.topic,
                msg_type: 'sensor_msgs/msg/Image'
            }
        }
        ws.sendMessage(JSON.stringify(msg))
        setPauseDisabled(true)
        setControls({...controls, paused: !controls.paused})
    }

    return (
        <Box>
            <Box>
                <Grid container alignItems="center" wrap="nowrap">
                    <Grid item>
                        <Typography variant="h6">
                            Topic
                        </Typography>
                    </Grid>
                    <Grid item>
                        <Box ml={2}>
                            <TextField value={typedTopic} onChange={handleTypedTopicChange}
                            sx={{minWidth: 300}} />
                        </Box>
                    </Grid>
                    <Grid item>
                        <Box ml={1}>
                            <BluePrimaryIconButton onClick={handleTopicChange}
                                disabled={syncDisabled}>
                                <SyncIcon />
                            </BluePrimaryIconButton>
                        </Box>
                    </Grid>
                </Grid>
            </Box>
            <Box pl="66px" mt={3}>
                <BluePrimaryButton sx={{minWidth: 300}} onClick={handlePauseChange}
                    endIcon={controls.paused ? <PlayArrowIcon /> : <PauseIcon />}
                    disabled={pauseDisabled}>
                    {controls.paused ? 'Play Stream' : 'Pause Stream'}
                </BluePrimaryButton>
            </Box>
        </Box>
    )
}