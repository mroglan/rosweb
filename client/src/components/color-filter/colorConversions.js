import convert from 'color-convert'

export const colorBounds = {
    HSV: {
        js: [360, 100, 100],
        cv2: [180, 255, 255]
    },
    RGB: {
        js: [255, 255, 255],
        cv2: [255, 255, 255]
    }
}

export function cv2Convert(color, encoding, targetEncoding) {
    color = color.map((c, i) => (c / colorBounds[encoding].cv2[i]) * colorBounds[encoding].js[i])
    color = convert[encoding.toLowerCase()][targetEncoding.toLowerCase()](color)
    color = color.map((c, i) => (c / colorBounds[targetEncoding].js[i]) * colorBounds[targetEncoding].cv2[i])
    return color
}