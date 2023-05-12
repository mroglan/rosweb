import { Box, Container, Typography } from "@mui/material";
import Main from "../components/color-filter/Main";

export default function ColorFilter({ws}) {

    return (
        <Box>
            <Box mx={3}>
                <Box mt={3} textAlign="center">
                    <Typography variant="h4">
                        Color Filter
                    </Typography>
                </Box>
                <Box mt={3}>
                    <Main ws={ws} />
                </Box>
            </Box>
        </Box>
    )
}