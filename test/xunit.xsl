<?xml version="1.0" encoding="utf-8"?>
<xsl:stylesheet version="2.0"
                xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
                xmlns:str="http://exslt.org/strings"
                extension-element-prefixes="str">

<xsl:template match="/testsuite">
    <html>
    <body>
        <style type="text/css">
            body {
                /*max-width: 1200px;*/
                /*margin: auto;*/
                margin-left: 5%;
                margin-right: 10%;
                background-color: #446F6F;
                color: #121D1D;
                font-family: "Merriweather Sans", sans-serif;
            }

            section {
                background-color: white;
                margin-top: 1em;
                padding: 0em 0em 1em 0em;
                border-left: 4px solid #A8C2C2;
                box-shadow: 0px 0px 10px 2px black;
            }

            section.good {
                border-left-color: #18AF18;
            }

            section.bad-true {
                border-left-color: #DB1D1D;
            }

            section.bad-true.errwhole header {
                background-color: #DB1D1D;
            }

            section:last-child:after {
                display: block;
                clear: both;
            }

            header {
                margin: 0 0 0.5em 0;
                padding: 1em 1em 0.5em 1em;
                background-color: #A8C2C2;
                color: black;
                font-family: Merriweather, serif;
                font-weight: bold;
            }

            .entry, .message {
                padding: 0 1em 0 1em;
            }

            .entry, .message, .foldable {
                clear: both;
            }

            .entry > .value {
                display: block;
                float: right;
            }

            th, td {
                text-align: left;
                padding: 0 1em 0 1em;
                font-weight: normal;
            }
            table {
                width: 100%;
            }
            td {
                text-align: right;
            }
            tr.bad-true > td, .bad-true.message {
                font-weight: bold;
                color: #BD1515;
            }

            tr.good > td, .good.message {
                font-weight: bold;
                color: #109710;
            }

            .foldable > .control {
                padding: 0.5em 1em 0 1em;
                text-decoration: underline;
                color: #118383;
            }

            .foldable > .control:hover {
                color: #44C3C3;
                cursor: pointer;
            }

            .foldable > .contents {
                padding: 0.5em 1em 0.5em 1em;
                margin: 0.5em 0 0 0;
                background-color: #CDD3D3;
                color: black;
                font-family: Consolas, "Courier New", Courier, monospace;
                font-size: small;
                overflow-x: auto;
                white-space: nowrap;
            }
            .arrow {
                /*margin-left: 0.5em;*/
                font-size: 75%;
                margin-right: 0.25em;
            }
        </style>
        <script type="text/javascript">
            function getFirstChildWithClass(clazz, parent) {
                return document.evaluate("./*[contains(concat(' ', normalize-space(@class), ' '), ' "+clazz+" ')]", parent, null, XPathResult.FIRST_ORDERED_NODE_TYPE, null).singleNodeValue;
            }
            function toggleVisible(ctl, target) {
                var arrow = getFirstChildWithClass('arrow', ctl);
                if (target.style.display != 'none') {
                    target.style.display = 'none';
                    arrow.textContent = '&#x25b6;';
                } else {
                    target.style.display = 'block';
                    arrow.textContent = '&#x25bc;';
                }
            }
        </script>

        <!-- Summary section -->
        <section class="errwhole bad-{@errors &gt; 0 or @failures &gt; 0}">
            <header>Summary</header>
            <table>
                <tr><th>Tests:</th><td><xsl:value-of select="@tests"/></td></tr>
                <tr class="bad-{@errors &gt; 0}"><th>Errors:</th><td><xsl:value-of select="@errors"/></td></tr>
                <tr class="bad-{@failures &gt; 0}"><th>Failures:</th><td><xsl:value-of select="@failures"/></td></tr>
                <tr><th>Skipped:</th><td><xsl:value-of select="@skipped"/></td></tr>
            </table>
        </section>

        <!-- Failed tests (if present) -->
        <xsl:if test="testcase/failure">
            <section class="bad-true">
                <header>Failed tests</header>
                <xsl:for-each select="testcase[failure]">
                    <xsl:sort select="concat(@classname, '.', @name)" />
                    <xsl:apply-templates select='.' />
                </xsl:for-each>
            </section>
        </xsl:if>

        <!-- Passed tests tests -->
        <section class="good">
            <header>Passed tests</header>
            <xsl:for-each select="testcase[not(failure)]">
                <xsl:sort select="concat(@classname, '.', @name)" />
                <xsl:apply-templates select='.' />
            </xsl:for-each>
        </section>

        <script type="text/javascript">
            document.querySelectorAll('.foldable').forEach((e) => {
                var ctl = getFirstChildWithClass('control', e);
                var target = getFirstChildWithClass('contents', e);
                ctl.onclick = function() {toggleVisible(ctl, target);};
                ctl.click();
            });
        </script>
    </body>
    </html>
</xsl:template>

<!-- Transform one test case -->
<xsl:template match="testcase">
    <xsl:variable name="clazz">
        <xsl:choose>
            <xsl:when test='./failure'>bad-true</xsl:when>
            <xsl:otherwise>good</xsl:otherwise>
        </xsl:choose>
    </xsl:variable>
    <div class="{$clazz} foldable testcase">
        <div class="control">
            <div style="float:left" class="arrow"></div>
            <div style="float:left">
                <xsl:value-of select="@classname"/>.<xsl:value-of select="@name"/>
            </div>
            <div style="float:right">
                <xsl:value-of select="@time"/>
            </div>
            <div style="clear:both"></div>
        </div>
        <div class="contents">
            <xsl:apply-templates select="system-out"/>
        </div>
    </div>
</xsl:template>

<!-- Strip absolute paths from test output -->
<xsl:template match="system-out">
    <xsl:for-each select='str:tokenize(./text(), "&#10;")'>
        <xsl:choose>
            <xsl:when test="contains(., 'CTU_CAN_FD/')"><xsl:value-of select="substring-after(., 'CTU_CAN_FD/')" /></xsl:when>
            <xsl:when test="contains(., 'CAN_FD_IP_Core/')"><xsl:value-of select="substring-after(., 'CAN_FD_IP_Core/')" /></xsl:when>
            <xsl:otherwise><xsl:value-of select="." /></xsl:otherwise>
        </xsl:choose><br />
    </xsl:for-each>
</xsl:template>

</xsl:stylesheet>
