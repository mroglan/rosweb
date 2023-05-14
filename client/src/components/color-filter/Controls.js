import { Autocomplete, Box, Grid, MenuItem, Select, TextField, Typography, useMediaQuery } from "@mui/material";
import { useMemo, useState, Fragment } from "react";
import SyncIcon from '@mui/icons-material/Sync';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import PauseIcon from '@mui/icons-material/Pause';
import DeleteIcon from '@mui/icons-material/Delete';
import { BluePrimaryButton, BluePrimaryIconButton, RedPrimaryIconButton } from "../misc/buttons";
import { colorBounds, cv2Convert } from './colorConversions'

const filterTypes = [
    'HSV',
    'RGB'
].sort()

export default function Controls({ws, controls, setControls}) {

    const [typedTopic, setTypedTopic] = useState('')
    const [typedFilter, setTypedFilter] = useState(controls.selectedFilter)

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

    const handleFilterChange = (e, val) => {
        if (!val) val = 'None'

        if (controls.filters.hasOwnProperty(val) || val == 'None') {
            setControls({...controls, selectedFilter: val})
        } else {
            setControls({...controls, selectedFilter: val, filters: {...controls.filters, [val]: {
                type: 'HSV',
                lower: [0, 0, 0],
                upper: colorBounds.HSV.cv2
            }}})
        }
    }
    
    const handleFilterTypeChange = (e) => {
        const lower = cv2Convert(controls.filters[controls.selectedFilter].lower, 
            controls.filters[controls.selectedFilter].type, e.target.value)
        const upper = cv2Convert(controls.filters[controls.selectedFilter].upper,
            controls.filters[controls.selectedFilter].type, e.target.value)
        setControls({...controls, filters: {...controls.filters, 
            [controls.selectedFilter]: {lower, upper, type: e.target.value}}})
    }

    const updateFilterBound = (bound, index, value) => {
        value = Number(value)
        if (value < 0) return
        if (value > colorBounds[controls.filters[controls.selectedFilter].type].cv2[index]) return

        const copy = [...controls.filters[controls.selectedFilter][bound]]
        copy[index] = value
        setControls({...controls, filters: {...controls.filters, [controls.selectedFilter]: {
            ...controls.filters[controls.selectedFilter],
            [bound]: copy
        }}})
    }

    return (
        <Box sx={{
            display: 'grid',
            gridTemplateColumns: 'auto 1fr auto',
            gridAutoFlow: 'row',
            alignItems: 'center'
        }}>
            <Box>
                <Typography variant="h6">
                    Topic
                </Typography>
            </Box>
            <Box ml={2}>
                <TextField value={typedTopic} onChange={handleTypedTopicChange}
                sx={{minWidth: 300}} />
            </Box>
            <Box ml={1}>
                <BluePrimaryIconButton onClick={handleTopicChange}
                    disabled={syncDisabled}>
                    <SyncIcon />
                </BluePrimaryIconButton>
            </Box>
            <Box />
            <Box ml={2} mt={1}>
                <BluePrimaryButton sx={{minWidth: 300}} onClick={handlePauseChange}
                    endIcon={controls.paused ? <PlayArrowIcon /> : <PauseIcon />}
                    disabled={pauseDisabled}>
                    {controls.paused ? 'Play Stream' : 'Pause Stream'}
                </BluePrimaryButton>
            </Box>
            <Box />
            <Box mt={3}>
                <Typography variant="h6">
                    Filter
                </Typography>
            </Box>
            <Box mt={3} ml={2}>
                <Autocomplete inputValue={typedFilter} onInputChange={(e, val) => setTypedFilter(val)}
                    value={controls.selectedFilter} onChange={handleFilterChange}
                    options={[...Object.keys(controls.filters), 'None']} sx={{minWidth: 300}}
                    renderInput={(params) => <TextField {...params} />} freeSolo />
            </Box>
            <Box mt={3}>
                {controls.selectedFilter !== 'None' && <RedPrimaryIconButton>
                    <DeleteIcon />     
                </RedPrimaryIconButton>}
            </Box>
            {controls.selectedFilter !== 'None' && <>
                <Box mt={3}>
                    <Typography variant="h6">
                        Type
                    </Typography>
                </Box> 
                <Box mt={3} ml={2}>
                    <Select value={controls.filters[controls.selectedFilter].type}
                        onChange={handleFilterTypeChange}
                        sx={{minWidth: 300}}>
                        {filterTypes.map(type => (
                            <MenuItem key={type} value={type}>{type}</MenuItem>
                        ))}
                    </Select>
                </Box>
                <Box />
                {controls.filters[controls.selectedFilter].type.split('').map((letter, i) => (
                    <Fragment key={i}>
                        <Box mt={3}>
                            <Typography variant="h6">
                                {letter}
                            </Typography>
                        </Box>
                        <Box mt={3} ml={2} sx={{gridColumn: 'span 2'}}>
                            <Grid container alignItems="center">
                                <Grid item>
                                    <TextField type="number" placeholder="0"
                                        value={controls.filters[controls.selectedFilter].lower[i] || ''}
                                        onChange={(e) => updateFilterBound('lower', i, e.target.value)}
                                        sx={{maxWidth: 100}} />
                                </Grid>
                                <Grid item>
                                    <Box mx={2}>
                                        <Typography varaint="body1">
                                            to
                                        </Typography>
                                    </Box>
                                </Grid>
                                <Grid item>
                                    <TextField type="number" placeholder="0"
                                        value={controls.filters[controls.selectedFilter].upper[i] || ''}
                                        onChange={(e) => updateFilterBound('upper', i, e.target.value)}
                                        sx={{maxWidth: 100}} />
                                </Grid>
                            </Grid>
                        </Box>
                    </Fragment>
                ))}
            </>}
        </Box>
    )
}