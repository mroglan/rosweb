import { Box, Paper, Typography } from "@mui/material";
import { Link } from "react-router-dom";

const options = [
    {name: 'Bagged Image to Video', link: '/bagged-image-to-video'},
    {name: 'Color Filter', link: '/color-filter'},
    {name: 'Waypointing', link: '/waypointing'}
].sort((a,b) => a.name.localeCompare(b.name))

export default function Options() {

    return (
        <Box>
            <Box maxWidth="sm" mx="auto">
                {options.map((option, i) => (
                    <Box key={i} my={1}>
                        <Paper elevation={3}>
                            <Link to={option.link}>
                                <Box p={1} textAlign="center">
                                    <Typography variant="h6">
                                        {option.name}
                                    </Typography>
                                </Box>
                            </Link>
                        </Paper>
                    </Box>
                ))}
            </Box>
        </Box>
    )
}