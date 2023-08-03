import React from 'react'
import {Box, Typography} from '@mui/material'
import Main from '../components/waypointing/Main'

export default function Waypointing({ws}) {
    return (
        <Box>
            <Box mx={3}>
                <Box mt={3} textAlign="center">
                    <Typography variant="h4">
                        Waypointing
                    </Typography>
                </Box>
                <Box mt={3}>
                    <Main ws={ws} />
                </Box>
            </Box>
        </Box>
    )
}