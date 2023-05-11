import { Box, Container, Typography } from "@mui/material";
import Main from '../components/bagged-image-to-video/Main'

export default function BaggedImageToVideo({ws}) {

    return (
        <Box>
            <Container maxWidth="md">
                <Box mt={3} textAlign="center">
                    <Typography variant="h4">
                        Bagged Image to Video
                    </Typography>
                </Box>
                <Box mt={3}>
                    <Main ws={ws} />
                </Box>
            </Container>
        </Box>
    )
}