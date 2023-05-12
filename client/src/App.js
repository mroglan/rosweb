import React, { useMemo } from 'react'
import {BrowserRouter, Routes, Route, Link} from 'react-router-dom'
import Home from './pages'
import BaggedImageToVideo from './pages/bagged-image-to-video'
import useWebSocket from 'react-use-websocket'
import { Box, Typography } from '@mui/material'
import NoConnection from './components/index/NoConnection'
import ColorFilter from './pages/color-filter'

export const SOCKET_URL = 'ws://localhost:8083'

export default function App() {

	const ws = useWebSocket(SOCKET_URL, {
		shouldReconnect: (e) => true,
		reconnectAttempts: 100,
		reconnectInterval: 1000
	})

	useMemo(() => {
		// TODO: 
		// reset anything in the ros_session
		console.log('reloaded application')
	}, [])

	return (
		<BrowserRouter>
			<NoConnection readyState={ws.readyState} />
			<Box textAlign="center">
				<Link to="/">
					<Typography color="primary.main" variant="h2">
						RosWeb
					</Typography>
				</Link>
			</Box>
			<Routes>
				<Route path="/" element={<Home />} />
				<Route path="/bagged-image-to-video" element={<BaggedImageToVideo ws={ws} />} />
				<Route path="/color-filter" element={<ColorFilter ws={ws} />} />
			</Routes>	
		</BrowserRouter>
	)
}
