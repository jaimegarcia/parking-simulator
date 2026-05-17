# 2D Parking Simulator

A static HTML parking simulator with clickable parking spots, randomized obstacles, and automatic parking path planning.

## Run

Open `index.html` directly in a browser, or start a small local server:

```bash
npm start
```

Then visit:

```text
http://localhost:4173
```

## Deploy to GitHub Pages

This project is a static site, so it can be deployed directly from the repository root.

1. Push the project to GitHub on the `main` branch.
2. In the GitHub repository, open **Settings > Pages**.
3. Set **Build and deployment > Source** to **GitHub Actions**.
4. Push to `main`, or run the **Deploy to GitHub Pages** workflow manually from the **Actions** tab.

The deployment workflow runs `npm test`, uploads the repository root as the Pages artifact, and publishes `index.html`, `styles.css`, and `app.js`.
