import React from 'react'
import {Box, Container, Paper, Typography} from '@mui/material'
import Options from '../components/index/Options'

export default function Home() {
    return (
        <Box>
            <Container maxWidth="md">
                <Box mt={3} textAlign="center">
                    <Typography variant="h4">
                        Available Tools
                    </Typography>
                </Box>
                <Box mt={3}>
                    <Options />
                </Box>
            </Container>
        </Box>
    )
}