import { Box, Typography } from '@mui/material';
import { ReadyState } from 'react-use-websocket'

export default function NoConnection({readyState}) {

    if (readyState == ReadyState.OPEN) return;

    return (
        <Box textAlign="center" bgcolor="error.light">
            <Typography color="text.primary">
                No connection to server.
            </Typography>
        </Box>
    )
}