import { Box, TextField, Typography } from "@mui/material";
import { useState, useMemo } from "react";
import { BluePrimaryIconButton } from "../misc/buttons";
import SyncIcon from '@mui/icons-material/Sync';

export default function Settings({ws, controls, setControls}) {

    const [typedGpsTopic, setTypedGpsTopic] = useState(controls.gpsTopic)
    const [typedOdomTopic, setTypedOdomTopic] = useState(controls.odomTopic)

    const [syncDisabled, setSyncDisabled] = useState(false)

    useMemo(() => {
        setSyncDisabled(false)
    }, [controls])

    const handleTypedGpsTopicChange = (e) => {
        setTypedGpsTopic(e.target.value)
    }

    const handleTypedOdomTopicChange = (e) => {
        setTypedOdomTopic(e.target.value)
    }

    const handleTopicChange = (controlsTopic, topic, msgType) => {
        if (topic == controls[controlsTopic]) return

        let msg = {
            type: 'request',
            data: {
                msg_type: msgType
            }
        }
        if (controls[controlsTopic]) {
            msg.operation = 'change_subscriber'
            msg.data.prev_topic_name = controls[controlsTopic]
            msg.data.new_topic_name = topic
        } else {
            msg.operation = 'create_subscriber'
            msg.data.topic_name = topic
        }
        console.log('sending', msg)
        ws.sendMessage(JSON.stringify(msg))
        setSyncDisabled(true)
        setControls({...controls, [controlsTopic]: topic})
    }

    const handleGpsTopicChange = (e) => {
        handleTopicChange('gpsTopic', typedGpsTopic, 'sensor_msgs/msg/NavSatFix')
    }

    const handleOdomTopicChange = (e) => {
        handleTopicChange('odomTopic', typedOdomTopic, 'nav_msgs/msg/Odometry')
    }

    return (
        <Box>
            <Box sx={{
                display: 'grid',
                gridTemplateColumns: 'auto 1fr auto',
                gridAutoFlow: 'row',
                alignItems: 'center'
            }}>
                <Box>
                    <Typography variant="h6">
                        GPS Topic
                    </Typography>
                </Box>
                <Box ml={2}>
                    <TextField value={typedGpsTopic} onChange={handleTypedGpsTopicChange} 
                        fullWidth />
                </Box>
                <Box ml={1}>
                    <BluePrimaryIconButton onClick={handleGpsTopicChange}
                        disabled={syncDisabled}>
                        <SyncIcon />
                    </BluePrimaryIconButton>
                </Box>
                <Box mt={3}>
                    <Typography variant="h6">
                        Odom Topic
                    </Typography>
                </Box>
                <Box ml={2} mt={3}>
                    <TextField value={typedOdomTopic} onChange={handleTypedOdomTopicChange} 
                        fullWidth />
                </Box>
                <Box ml={1} mt={3}>
                    <BluePrimaryIconButton onClick={handleOdomTopicChange}
                        disabled={syncDisabled}>
                        <SyncIcon />
                    </BluePrimaryIconButton>
                </Box>
            </Box>
        </Box>
    )
}