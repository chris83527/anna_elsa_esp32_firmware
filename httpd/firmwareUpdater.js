/* 
 * The MIT License
 *
 * Copyright 2023 chris.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

function startUpload() {
    var otafile = document.getElementById("otafile").files;

    if (otafile.length === 0) {
        alert("No file selected!");
    } else {
        document.getElementById("otafile").disabled = true;
        document.getElementById("upload").disabled = true;

        var file = otafile[0];
        var xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function () {
            if (xhr.readyState === 4) {
                if (xhr.status === 200) {
                    document.open();
                    document.write(xhr.responseText);
                    document.close();
                } else if (xhr.status === 0) {
                    alert("Server closed the connection abruptly!");
                    location.reload();
                } else {
                    alert(xhr.status + " Error!\n" + xhr.responseText);
                    location.reload();
                }
            }
        };

        xhr.upload.onprogress = function (e) {
            var progress = document.getElementById("progress");
            //progress.textContent = "Progress: " + (e.loaded / e.total * 100).toFixed(0) + "%";
            progress.ariaValueNow = (e.loaded / e.total * 100).toFixed(0);
        };
        xhr.open("POST", "/update", true);
        xhr.send(file);
    }
}
